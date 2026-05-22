# COMPASS user manual build.
#
# Targets:
#   draft    - fast incremental build via latexmk (default; used by auto-build)
#   quality  - clean full rebuild, all passes settled (use before releases)
#   clean    - remove auxiliary files

JOBNAME  := user_manual
LATEXMK  := latexmk
PDFLATEX := pdflatex

PDFLATEX_FLAGS := -interaction=nonstopmode -file-line-error -synctex=1

# Pick up the local texmf.cnf (raised memory limits for this large document).
# The trailing :: keeps the system texmf.cnf in the search path.
export TEXMFCNF := $(CURDIR)::

.PHONY: draft quality clean help

help:
	@echo "Targets:"
	@echo "  draft    - fast incremental build (default workflow)"
	@echo "  quality  - clean full rebuild (use before releases)"
	@echo "  clean    - remove auxiliary files"

draft:
	$(LATEXMK) -pdf -pdflatex="$(PDFLATEX) $(PDFLATEX_FLAGS) %O %S" -f $(JOBNAME)

quality: clean
	$(LATEXMK) -pdf -pdflatex="$(PDFLATEX) $(PDFLATEX_FLAGS) %O %S" -g $(JOBNAME)

clean:
	$(LATEXMK) -C $(JOBNAME) || true
	rm -f $(JOBNAME).ego $(JOBNAME).ent $(JOBNAME).glo $(JOBNAME).gls \
	      $(JOBNAME).ilg $(JOBNAME).ind lines.idx