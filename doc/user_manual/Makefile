# COMPASS user manual build.
#
# Targets:
#   draft    - fast incremental build via latexmk (default; used by auto-build)
#   quality  - clean full rebuild, all passes settled (use before releases)
#   clean    - remove auxiliary files

# user_manual and user_manual_appendix read each other's .aux (xr-hyper), so each
# build runs latexmk twice: the second call picks up the labels of the other document.
JOBNAMES := user_manual_appendix user_manual
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
	$(LATEXMK) -pdf -pdflatex="$(PDFLATEX) $(PDFLATEX_FLAGS) %O %S" -f $(JOBNAMES)
	$(LATEXMK) -pdf -pdflatex="$(PDFLATEX) $(PDFLATEX_FLAGS) %O %S" -f $(JOBNAMES)

quality: clean
	$(LATEXMK) -pdf -pdflatex="$(PDFLATEX) $(PDFLATEX_FLAGS) %O %S" -g $(JOBNAMES)
	$(LATEXMK) -pdf -pdflatex="$(PDFLATEX) $(PDFLATEX_FLAGS) %O %S" $(JOBNAMES)

clean:
	$(LATEXMK) -C $(JOBNAMES) || true
	for j in $(JOBNAMES); do \
	    rm -f $$j.ego $$j.ent $$j.glo $$j.gls $$j.ilg $$j.ind; \
	done
	rm -f lines.idx