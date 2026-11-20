# COMPASS user manual build.
#
# Targets:
#   draft    - fast incremental build via latexmk (default; used by auto-build)
#   quality  - clean full rebuild, all passes settled (use before releases)
#   clean    - remove the build folder and the copied PDFs
#
# Output: user_manual_v<version><SUFFIX>.pdf and user_manual_appendix_v<version><SUFFIX>.pdf,
# version from CMakeLists.txt, optional SUFFIX for hotfixes, e.g. make draft SUFFIX=_hf4.
# The two PDFs link to each other by these file names, so they must not be renamed.
# All auxiliary files stay in build/, the PDFs are copied from there into this folder
# (copied, not moved, so latexmk keeps its incremental state).

VERSION    := $(shell grep -oP 'CPACK_PACKAGE_VERSION_(MAJOR|MINOR|PATCH) "\K[0-9]+' ../../CMakeLists.txt | paste -sd.)
SUFFIX     ?=
FILESUFFIX := _v$(VERSION)$(SUFFIX)

# user_manual and user_manual_appendix read each other's .aux (xr-hyper), so each
# build runs latexmk twice: the second pass picks up the labels of the other document.
DOCS     := user_manual_appendix user_manual
PDFS     := $(foreach d,$(DOCS),$(d)$(FILESUFFIX).pdf)
# also hardcoded in the clean target, so that rm -rf never gets an empty path
BUILDDIR := build
LATEXMK  := latexmk
PDFLATEX := pdflatex

PDFLATEX_FLAGS := -interaction=nonstopmode -file-line-error -synctex=1
PRETEX         := \def\compassversion{$(VERSION)}\def\compassfilesuffix{$(FILESUFFIX)}
LATEXMK_RUN     = $(LATEXMK) -pdf -outdir=$(BUILDDIR) -pdflatex="$(PDFLATEX) $(PDFLATEX_FLAGS) %O %P" -usepretex='$(PRETEX)'

# Pick up the local texmf.cnf (raised memory limits for this large document).
# The trailing :: keeps the system texmf.cnf in the search path.
export TEXMFCNF := $(CURDIR)::

.PHONY: draft quality clean help check_version

help:
	@echo "Targets:"
	@echo "  draft    - fast incremental build (default workflow)"
	@echo "  quality  - clean full rebuild (use before releases)"
	@echo "  clean    - remove the build folder and the copied PDFs"
	@echo "Output: $(PDFS)"

check_version:
	@echo "$(VERSION)" | grep -qP '^\d+\.\d+\.\d+$$' || { echo "no version in ../../CMakeLists.txt"; exit 1; }

draft: check_version
	for pass in 1 2; do for d in $(DOCS); do \
	    $(LATEXMK_RUN) -f -jobname=$${d}$(FILESUFFIX) $${d}.tex || exit 1; \
	done; done
	cp $(addprefix $(BUILDDIR)/,$(PDFS)) .

quality: clean
	for d in $(DOCS); do $(LATEXMK_RUN) -g -jobname=$${d}$(FILESUFFIX) $${d}.tex || exit 1; done
	for d in $(DOCS); do $(LATEXMK_RUN) -jobname=$${d}$(FILESUFFIX) $${d}.tex || exit 1; done
	cp $(addprefix $(BUILDDIR)/,$(PDFS)) .

clean: check_version
	rm -rf build
	rm -f $(PDFS)
