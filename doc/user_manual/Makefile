# COMPASS user manual build.
#
# Targets:
#   draft    - fast incremental build via latexmk (default; used by auto-build)
#   quality  - clean full rebuild, all passes settled (use before releases)
#   clean    - remove auxiliary files
#
# Output: user_manual_v<version><SUFFIX>.pdf and user_manual_appendix_v<version><SUFFIX>.pdf,
# version from CMakeLists.txt, optional SUFFIX for hotfixes, e.g. make draft SUFFIX=_hf4.
# The two PDFs link to each other by these file names, so they must not be renamed.

VERSION    := $(shell grep -oP 'CPACK_PACKAGE_VERSION_(MAJOR|MINOR|PATCH) "\K[0-9]+' ../../CMakeLists.txt | paste -sd.)
SUFFIX     ?=
FILESUFFIX := _v$(VERSION)$(SUFFIX)

# user_manual and user_manual_appendix read each other's .aux (xr-hyper), so each
# build runs latexmk twice: the second pass picks up the labels of the other document.
DOCS     := user_manual_appendix user_manual
LATEXMK  := latexmk
PDFLATEX := pdflatex

PDFLATEX_FLAGS := -interaction=nonstopmode -file-line-error -synctex=1
PRETEX         := \def\compassversion{$(VERSION)}\def\compassfilesuffix{$(FILESUFFIX)}
LATEXMK_RUN     = $(LATEXMK) -pdf -pdflatex="$(PDFLATEX) $(PDFLATEX_FLAGS) %O %P" -usepretex='$(PRETEX)'

# Pick up the local texmf.cnf (raised memory limits for this large document).
# The trailing :: keeps the system texmf.cnf in the search path.
export TEXMFCNF := $(CURDIR)::

.PHONY: draft quality clean help check_version

help:
	@echo "Targets:"
	@echo "  draft    - fast incremental build (default workflow)"
	@echo "  quality  - clean full rebuild (use before releases)"
	@echo "  clean    - remove auxiliary files"
	@echo "Output: $(foreach d,$(DOCS),$(d)$(FILESUFFIX).pdf)"

check_version:
	@echo "$(VERSION)" | grep -qP '^\d+\.\d+\.\d+$$' || { echo "no version in ../../CMakeLists.txt"; exit 1; }

draft: check_version
	for pass in 1 2; do for d in $(DOCS); do \
	    $(LATEXMK_RUN) -f -jobname=$${d}$(FILESUFFIX) $${d}.tex || exit 1; \
	done; done

quality: clean
	for d in $(DOCS); do $(LATEXMK_RUN) -g -jobname=$${d}$(FILESUFFIX) $${d}.tex || exit 1; done
	for d in $(DOCS); do $(LATEXMK_RUN) -jobname=$${d}$(FILESUFFIX) $${d}.tex || exit 1; done

clean: check_version
	for d in $(DOCS); do \
	    $(LATEXMK) -C -jobname=$${d}$(FILESUFFIX) $${d}.tex || true; \
	    rm -f $${d}$(FILESUFFIX).ego $${d}$(FILESUFFIX).ent $${d}$(FILESUFFIX).glo $${d}$(FILESUFFIX).gls \
	          $${d}$(FILESUFFIX).ilg $${d}$(FILESUFFIX).ind; \
	done
	rm -f lines.idx
