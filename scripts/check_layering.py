#!/usr/bin/env python3
"""
Check the dependencies between the COMPASS libraries against the target layering.

Every source file in src/ is mapped to the library it belongs to in the target layering of
readme_dependencies.md. A dependency on a library on the same or a higher layer is a violation.

Include check (two modes):
- exact (--build-dir): reads the headers each compiled file actually includes from the build
  directory (`ninja -t deps`, or compiler_depend.make / depend.make of the Makefile generator).
  The compiled file is attributed to the library of its CMake target, so generated files count too.
- text (no build directory): resolves the #include lines of src/ by path suffix against the
  headers in src/. Approximate, since it resolves by file name.
Known include violations are listed in the allowlist; the check fails on any violation not listed
there and on any allowlist entry that no longer occurs, so the list can only shrink.

CMake check (always): target_link_libraries and target_include_directories of the host libraries
must not point to the same or a higher layer, and no global include_directories may point into src/.

Includes into experimental_src/ are not checked; they belong to the plugin plan and are only reported.

Usage:
    python3 scripts/check_layering.py                    text mode, check against the allowlist
    python3 scripts/check_layering.py --build-dir build  exact mode on the given build directory
    python3 scripts/check_layering.py --write-allowlist  replace the allowlist with the current violations
    python3 scripts/check_layering.py --matrix           print the library include matrix (text mode)
    python3 scripts/check_layering.py --experimental     list the includes into experimental_src/ (text mode)
    python3 scripts/check_layering.py --graph deps.svg   write the library dependency graph (.dot, or
                                                         .svg/.png/.pdf rendered with Graphviz 'dot')
    python3 scripts/check_layering.py --graph deps.svg --graph-reduced
                                                         same, without edges implied by other paths
"""

import argparse
import collections
import fnmatch
import glob
import math
import os
import re
import subprocess
import sys

REPO_ROOT      = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_LIST   = os.path.join(REPO_ROOT, "scripts", "layering_allowlist.txt")
SOURCE_DIR     = "src"
EXPERIMENT_DIR = "experimental_src"

# target layering, bottom to top; a library may only depend on lower layers
LAYERS = [
    ["util"],
    ["config"],
    ["core"],
    ["gui"],
    ["filter", "reconstruction"],
    ["view"],
    ["result"],
    ["eval"],
    ["task"],
    ["app"],
]

# path patterns (relative to the repo root) -> library, first match wins
LIBRARY_RULES = [
    # module directories
    ("src/util/*",                                      "util"),
    ("src/result/*",                                    "result"),
    ("src/app/*",                                       "app"),
    ("src/config/*",                                    "config"),
    ("src/core/*",                                      "core"),
    ("src/eval/*",                                      "eval"),
    ("src/filter/*",                                    "filter"),
    ("src/gui/*",                                       "gui"),
    ("src/reconstruction/*",                            "reconstruction"),
    ("src/task/*",                                      "task"),
    ("src/view/*",                                      "view"),
]

# CMake targets of the host libraries -> library
TARGET_LIBRARIES = {
    "compass_util":           "util",
    "compass_config":         "config",
    "compass_core":           "core",
    "compass_gui":            "gui",
    "compass_filter":         "filter",
    "compass_reconstruction": "reconstruction",
    "compass_view":           "view",
    "compass_result":         "result",
    "compass_eval":           "eval",
    "compass_task":           "task",
    "compass_app":            "app",
    "compass_client":         "app",
}

# CMake targets of experimental_src (plugin plan, not checked)
EXPERIMENTAL_TARGETS = {
    "compass_osg_utils", "compass_docx", "compass_reconst_adv",
    "compass_geo_view", "compass_rtcmd_ext", "compass_test_lab",
}

# files not checked
EXCLUDE_RULES = [
    "*/unit_tests/*",
    "src/app/watchdog/*",
]

# headers whose implementation lives in another directory => they belong to the library of the implementation
IMPLEMENTED_IN = {
}

INCLUDE_RE    = re.compile(r'^\s*#\s*include\s+"([^"]+)"')
OBJECT_RE     = re.compile(r'CMakeFiles/(compass_[a-z_]+)\.dir/')
CMAKE_CALL_RE = re.compile(r'\b(target_link_libraries|target_include_directories|include_directories)\s*\(([^)]*)\)', re.S)
SOURCE_EXT    = (".h", ".cpp")

LAYER_OF = {lib: idx for idx, libs in enumerate(LAYERS) for lib in libs}


def is_excluded(rel_path):
    return any(fnmatch.fnmatch(rel_path, pattern) for pattern in EXCLUDE_RULES)


def library_of(rel_path):
    """Library of a source file in the target layering, None if unmapped."""
    rel_path = IMPLEMENTED_IN.get(rel_path, rel_path)

    for pattern, lib in LIBRARY_RULES:
        if fnmatch.fnmatch(rel_path, pattern):
            return lib

    return None


def is_upward(from_lib, to_lib):
    """A dependency of from_lib on to_lib breaks the layering."""
    return to_lib != from_lib and LAYER_OF[to_lib] >= LAYER_OF[from_lib]


def collect_sources(root, top_dir, extensions=SOURCE_EXT):
    """All files with the given extensions below top_dir, as repo-relative paths with forward slashes."""
    sources = []

    for dirpath, _, filenames in os.walk(os.path.join(root, top_dir)):
        for filename in filenames:
            if filename.endswith(extensions):
                rel = os.path.relpath(os.path.join(dirpath, filename), root)
                sources.append(rel.replace(os.sep, "/"))

    return sorted(sources)


def index_headers(paths):
    """Header basename -> list of header paths."""
    index = collections.defaultdict(list)

    for path in paths:
        if path.endswith(".h"):
            index[os.path.basename(path)].append(path)

    return index


def resolve(include, including_file, header_index):
    """Candidate header paths for an include, preferring the including file's directory."""
    candidates = [path for path in header_index.get(os.path.basename(include), [])
                  if path == include or path.endswith("/" + include)]

    local = os.path.normpath(os.path.join(os.path.dirname(including_file), include)).replace(os.sep, "/")
    if local in candidates:
        return [local]

    return candidates


def scan_text(root):
    """Text mode. Return (edges, violations, experimental) for the current tree.

    edges:        (from_lib, to_lib) -> set of including files
    violations:   set of (including file, included header) pairs breaking the layering
    experimental: set of (including file, included header) pairs resolving into experimental_src/
    """
    sources          = collect_sources(root, SOURCE_DIR)
    header_index     = index_headers(sources)
    experiment_index = index_headers(collect_sources(root, EXPERIMENT_DIR))

    edges        = collections.defaultdict(set)
    violations   = set()
    experimental = set()

    for path in sources:
        if is_excluded(path):
            continue

        from_lib = library_of(path)
        if from_lib is None:
            continue

        with open(os.path.join(root, path), errors="ignore") as fh:
            for line in fh:
                match = INCLUDE_RE.match(line)
                if not match:
                    continue

                include    = match.group(1)
                candidates = [c for c in resolve(include, path, header_index) if not is_excluded(c)]

                if not candidates:
                    if resolve(include, path, experiment_index):
                        experimental.add((path, include))
                    continue

                to_libs = {library_of(c) for c in candidates} - {None}

                # ambiguous across libraries => cannot attribute, skip
                if len(to_libs) != 1:
                    continue

                to_lib = to_libs.pop()
                if to_lib == from_lib:
                    continue

                edges[(from_lib, to_lib)].add(path)

                if is_upward(from_lib, to_lib):
                    violations.add((path, candidates[0]))

    return edges, violations, experimental


def repo_relative(path, root):
    """Repo-relative path of a dependency path, also for paths of a build in another location
    (e.g. a Docker container); None for files outside the repository (system, Qt, build dir)."""
    path = path.replace("\\", "/")

    if os.path.isabs(path):
        prefix = os.path.realpath(root).replace(os.sep, "/") + "/"
        real   = os.path.realpath(path).replace(os.sep, "/")
        if real.startswith(prefix):
            return real[len(prefix):]

    for top in (SOURCE_DIR, EXPERIMENT_DIR):
        marker = "/" + top + "/"
        pos    = path.rfind(marker)
        if pos < 0 and path.startswith(top + "/"):
            pos = -1
        elif pos < 0:
            continue

        rel = path[pos + 1:]
        if os.path.exists(os.path.join(root, rel)):
            return rel

    return None


def read_ninja_deps(build_dir):
    """object -> list of dependency paths (first is the source), from `ninja -t deps`."""
    out  = subprocess.run(["ninja", "-C", build_dir, "-t", "deps"],
                          check=True, stdout=subprocess.PIPE, universal_newlines=True).stdout
    deps = {}
    obj  = None

    for line in out.splitlines():
        if not line.strip():
            obj = None
        elif not line[0].isspace():
            obj = line.split(":", 1)[0]
            deps[obj] = []
        elif obj is not None:
            deps[obj].append(line.strip())

    return deps


def read_make_deps(build_dir):
    """object -> list of dependency paths (first is the source), from the Makefile generator's
    compiler_depend.make / depend.make files."""
    deps = {}

    files = (glob.glob(os.path.join(build_dir, "**", "compiler_depend.make"), recursive=True) +
             glob.glob(os.path.join(build_dir, "**", "depend.make"), recursive=True))

    for path in files:
        with open(path, errors="ignore") as fh:
            text = fh.read().replace("\\\n", " ")

        for line in text.splitlines():
            if line.startswith("#") or ":" not in line:
                continue

            obj, _, rest = line.partition(": ")
            items = rest.split()
            if not obj.endswith(".o") or not items:
                continue

            deps.setdefault(obj.strip(), [])
            deps[obj.strip()].extend(items)

    return deps


def scan_build(root, build_dir):
    """Exact mode. Return (violations, edges, experimental) from the build directory.

    violations:   set of (compiled file, included header) pairs breaking the layering
    edges:        (from_lib, to_lib) -> set of compiled files using headers of to_lib (also transitively)
    experimental: from_lib -> set of compiled files using headers from experimental_src/
    """
    if os.path.exists(os.path.join(build_dir, "build.ninja")):
        deps = read_ninja_deps(build_dir)
    else:
        deps = read_make_deps(build_dir)

    if not deps:
        sys.exit("no dependency information found in '{}' (build it first)".format(build_dir))

    violations   = set()
    edges        = collections.defaultdict(set)
    experimental = collections.defaultdict(set)

    for obj, items in deps.items():
        match = OBJECT_RE.search(obj.replace("\\", "/"))
        if not match or match.group(1) not in TARGET_LIBRARIES:
            continue

        from_lib = TARGET_LIBRARIES[match.group(1)]
        source   = repo_relative(items[0], root) if items else None
        if source is None:
            source = obj  # generated file, e.g. moc compilation
        elif is_excluded(source):
            continue

        for item in items[1:]:
            header = repo_relative(item, root)
            if header is None or is_excluded(header):
                continue

            if header.startswith(EXPERIMENT_DIR + "/"):
                experimental[from_lib].add(source)
                continue

            to_lib = library_of(header)
            if to_lib is None or to_lib == from_lib:
                continue

            edges[(from_lib, to_lib)].add(source)

            if is_upward(from_lib, to_lib):
                violations.add((source, header))

    return violations, edges, experimental


def cmake_blocks(root):
    """(cmake file, command, arguments) of the link and include commands in the host CMake files."""
    files = [os.path.join(root, "CMakeLists.txt")] + [os.path.join(root, p)
             for p in collect_sources(root, SOURCE_DIR, ("CMakeLists.txt",))]

    for path in files:
        with open(path, errors="ignore") as fh:
            text = re.sub(r"#[^\n]*", "", fh.read())

        for match in CMAKE_CALL_RE.finditer(text):
            yield os.path.relpath(path, root), match.group(1), match.group(2).split()


def check_cmake(root):
    """CMake layering violations as list of (cmake file, message)."""
    violations = []

    for path, command, args in cmake_blocks(root):
        if command == "include_directories":
            # global include paths into src/ make every header reachable from every library
            for arg in args:
                arg = arg.strip('"')
                if (arg == "${CMAKE_SOURCE_DIR}" or "${CMAKE_SOURCE_DIR}/src" in arg or
                        (arg == "${CMAKE_CURRENT_LIST_DIR}" and path.startswith(SOURCE_DIR + "/"))):
                    violations.append((path, "global include path '{}'".format(arg)))
            continue

        if not args or args[0] not in TARGET_LIBRARIES:
            continue

        target   = args[0]
        from_lib = TARGET_LIBRARIES[target]

        for arg in args[1:]:
            arg = arg.strip('"')

            if command == "target_link_libraries":
                if arg in TARGET_LIBRARIES and arg != target and is_upward(from_lib, TARGET_LIBRARIES[arg]):
                    violations.append((path, "{} links {} ({} -> {})".format(target, arg, from_lib, TARGET_LIBRARIES[arg])))

            elif command == "target_include_directories" and "${CMAKE_SOURCE_DIR}/src/" in arg:
                to_lib = library_of(arg.replace("${CMAKE_SOURCE_DIR}/", "") + "/x")
                if to_lib is not None and is_upward(from_lib, to_lib):
                    violations.append((path, "{} includes {} ({} -> {})".format(target, arg, from_lib, to_lib)))

    return violations


def format_entry(entry):
    return "{} -> {}".format(entry[0], entry[1])


def read_allowlist(path):
    entries = set()

    if not os.path.exists(path):
        return entries

    with open(path) as fh:
        for line in fh:
            line = line.split("#", 1)[0].strip()
            if not line:
                continue

            src, sep, dst = line.partition(" -> ")
            if not sep:
                sys.exit("malformed allowlist line: '{}'".format(line))

            entries.add((src.strip(), dst.strip()))

    return entries


def write_allowlist(path, violations):
    with open(path, "w") as fh:
        fh.write("# Known violations of the target layering (readme_dependencies.md).\n")
        fh.write("# Generated by scripts/check_layering.py --write-allowlist. This list may only shrink.\n")
        fh.write("# Format: <including file> -> <included header>\n\n")

        for entry in sorted(violations):
            fh.write(format_entry(entry) + "\n")


def print_matrix(edges):
    libs  = [lib for layer in LAYERS for lib in layer]
    width = max(len(lib) for lib in libs) + 2

    print("from \\ to".ljust(width) + "".join(lib[:8].rjust(9) for lib in libs))

    for a in libs:
        cells = [str(len(edges[(a, b)])) if (a, b) in edges else "." for b in libs]
        print(a.ljust(width) + "".join(cell.rjust(9) for cell in cells))


def reduce_edges(edges):
    """Transitive reduction: drops an edge a -> b when b is also reachable from a over other edges.
    Layering violations are always kept, and they are not used as paths for the reduction."""
    adj = collections.defaultdict(set)
    for a, b in edges:
        if not is_upward(a, b):
            adj[a].add(b)

    def reachableOverOthers(a, b):
        todo = [n for n in adj[a] if n != b]
        seen = set(todo)
        while todo:
            n = todo.pop()
            if n == b:
                return True
            for m in adj[n] - seen:
                seen.add(m)
                todo.append(m)
        return False

    return {(a, b): files for (a, b), files in edges.items() if is_upward(a, b) or not reachableOverOthers(a, b)}


def write_graph(path, edges, experimental, title):
    """Write the library dependency graph, as DOT or rendered by Graphviz depending on the extension.

    edges:        (from_lib, to_lib) -> set of files, drawn with the file count as label
    experimental: from_lib -> set of files using experimental_src/, drawn dashed
    """
    lines = [
        "digraph compass_layering {",
        '    label="{}"; labelloc=t; fontname="Helvetica"; fontsize=14;'.format(title),
        "    rankdir=TB; nodesep=0.5; ranksep=0.7;",
        '    node [shape=box, style="rounded,filled", fillcolor="#e8eef7", color="#5b6b82", fontname="Helvetica"];',
        '    edge [fontname="Helvetica", fontsize=9, color="#7a8699", fontcolor="#5b6b82", arrowsize=0.7];',
    ]

    # one row per layer, top to bottom, kept in order by invisible edges
    for idx, libs in reversed(list(enumerate(LAYERS))):
        nodes = " ".join('"{0}" [label="{0}\\nlayer {1}"];'.format(lib, idx + 1) for lib in libs)
        lines.append("    {{ rank=same; {} }}".format(nodes))

    for upper, lower in zip(LAYERS[1:], LAYERS[:-1]):
        lines.append('    "{}" -> "{}" [style=invis, weight=10];'.format(upper[0], lower[0]))

    for (a, b), files in sorted(edges.items()):
        n     = len(files)
        attrs = 'label="{}", penwidth={:.1f}'.format(n, 0.6 + math.log2(n) / 3)

        # violations of the layering in red
        if is_upward(a, b):
            attrs += ', color="#d62728", fontcolor="#d62728", penwidth=2.5'

        lines.append('    "{}" -> "{}" [{}];'.format(a, b, attrs))

    if experimental:
        lines.append('    "{0}" [label="{0}\\n(not checked)", style="rounded,dashed", fillcolor=white];'.format(EXPERIMENT_DIR))
        for lib, files in sorted(experimental.items()):
            lines.append('    "{}" -> "{}" [label="{}", style=dashed, color="#b0b0b0", fontcolor="#909090", constraint=false];'
                         .format(lib, EXPERIMENT_DIR, len(files)))

    lines.append("}")
    dot_text = "\n".join(lines) + "\n"

    ext = os.path.splitext(path)[1].lower()

    if ext in (".dot", ".gv"):
        with open(path, "w") as fh:
            fh.write(dot_text)
    elif ext in (".svg", ".png", ".pdf"):
        try:
            subprocess.run(["dot", "-T" + ext[1:], "-o", path], input=dot_text,
                           check=True, universal_newlines=True)
        except FileNotFoundError:
            sys.exit("Graphviz 'dot' not found: install graphviz or write a .dot file instead")
    else:
        sys.exit("unsupported graph format '{}', use .dot, .svg, .png or .pdf".format(ext))


def print_summary(violations, from_lib_of):
    by_edge = collections.defaultdict(set)

    for src, dst in violations:
        by_edge[(from_lib_of(src), library_of(dst))].add(src)

    for (a, b), files in sorted(by_edge.items(), key=lambda kv: -len(kv[1])):
        print("  {:15s} -> {:15s} {:4d} files".format(str(a), str(b), len(files)))


def main():
    parser = argparse.ArgumentParser(description="Check the COMPASS library layering.")
    parser.add_argument("--allowlist", default=DEFAULT_LIST, help="allowlist file")
    parser.add_argument("--build-dir", help="build directory for the exact include check")
    parser.add_argument("--write-allowlist", action="store_true", help="replace the allowlist with the current violations")
    parser.add_argument("--matrix", action="store_true", help="print the library include matrix (text mode)")
    parser.add_argument("--experimental", action="store_true", help="list the includes into experimental_src/ (text mode)")
    parser.add_argument("--graph", metavar="FILE",
                        help="write the library dependency graph of the current mode (.dot, .svg, .png or .pdf)")
    parser.add_argument("--graph-reduced", action="store_true",
                        help="with --graph: omit edges implied by other paths (transitive reduction)")
    args = parser.parse_args()

    if args.graph_reduced and not args.graph:
        parser.error("--graph-reduced needs --graph FILE")

    if args.build_dir:
        violations, edges, experimental_by_lib = scan_build(REPO_ROOT, os.path.abspath(args.build_dir))
        from_lib_of  = lambda path: library_of(path) if path.startswith(SOURCE_DIR + "/") else "generated"
        experimental = None
        mode         = "exact ({})".format(args.build_dir)
        graph_title  = "COMPASS library dependencies, exact mode: compiled files using headers of the library (incl. transitive)"
    else:
        edges, violations, experimental = scan_text(REPO_ROOT)
        experimental_by_lib = collections.defaultdict(set)
        for path, _ in experimental:
            experimental_by_lib[library_of(path)].add(path)
        from_lib_of = library_of
        mode        = "text"
        graph_title = "COMPASS library dependencies, text mode: files including headers of the library"

    if args.graph:
        graph_edges = edges
        if args.graph_reduced:
            graph_edges  = reduce_edges(edges)
            graph_title += "\\nreduced: edges implied by other paths omitted ({} of {})".format(
                len(edges) - len(graph_edges), len(edges))

        write_graph(args.graph, graph_edges, experimental_by_lib, graph_title)
        print("wrote dependency graph to {}".format(args.graph))

    if args.matrix or args.experimental:
        text_edges, text_experimental = edges, experimental
        if experimental is None:
            text_edges, _, text_experimental = scan_text(REPO_ROOT)

        if args.matrix:
            print_matrix(text_edges)
            print()

        if args.experimental:
            print("includes into {}/ (not checked):".format(EXPERIMENT_DIR))
            for entry in sorted(text_experimental):
                print("  " + format_entry(entry))
            print()

    if args.write_allowlist:
        write_allowlist(args.allowlist, violations)
        print("wrote {} entries to {}".format(len(violations), os.path.relpath(args.allowlist, REPO_ROOT)))
        return 0

    allowed = read_allowlist(args.allowlist)
    new     = violations - allowed
    stale   = allowed - violations
    cmake   = check_cmake(REPO_ROOT)

    if experimental is None:
        num_objects       = len(set().union(*experimental_by_lib.values())) if experimental_by_lib else 0
        experimental_info = "host objects including {}/: {}".format(EXPERIMENT_DIR, num_objects)
    else:
        experimental_info = "includes into {}/: {}".format(EXPERIMENT_DIR, len(experimental))

    print("layering check, {} mode".format(mode))
    print("include violations: {} ({} allowlisted), {}".format(len(violations), len(violations & allowed), experimental_info))
    print_summary(violations, from_lib_of)
    print("cmake violations: {}".format(len(cmake)))

    if new:
        print("\nnew include violations (not in the allowlist):")
        for entry in sorted(new):
            print("  {}   [{} -> {}]".format(format_entry(entry), from_lib_of(entry[0]), library_of(entry[1])))

    if stale:
        print("\nallowlist entries that no longer occur (remove them, e.g. with --write-allowlist):")
        for entry in sorted(stale):
            print("  " + format_entry(entry))

    if cmake:
        print("\ncmake violations (link or include paths against the layering):")
        for path, message in cmake:
            print("  {}: {}".format(path, message))

    return 1 if new or stale or cmake else 0


if __name__ == "__main__":
    sys.exit(main())
