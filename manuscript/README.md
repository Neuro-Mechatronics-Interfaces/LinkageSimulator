# Building the technical manuscript

The manuscript entry point is [`main.tex`](main.tex), with references in [`main.bib`](main.bib). It requires a LaTeX distribution containing `pdflatex`, `bibtex`, and the standard `geometry`, `amsmath`, and `hyperref` packages. The project has been validated with TeX Live 2025.

## From the repository root

In PowerShell, run BibTeX between the first and final LaTeX passes:

```powershell
Push-Location manuscript
pdflatex -interaction=nonstopmode -halt-on-error main.tex
bibtex main
pdflatex -interaction=nonstopmode -halt-on-error main.tex
pdflatex -interaction=nonstopmode -halt-on-error main.tex
Pop-Location
```

The generated document is `manuscript/main.pdf`.

If `pdflatex` is not on `PATH`, use the TeX Live executable directly:

```powershell
Push-Location manuscript
& 'C:\texlive\2025\bin\windows\pdflatex.exe' -interaction=nonstopmode -halt-on-error main.tex
& 'C:\texlive\2025\bin\windows\bibtex.exe' main
& 'C:\texlive\2025\bin\windows\pdflatex.exe' -interaction=nonstopmode -halt-on-error main.tex
& 'C:\texlive\2025\bin\windows\pdflatex.exe' -interaction=nonstopmode -halt-on-error main.tex
Pop-Location
```

## Using LatexMk

When `latexmk` is available, it automatically performs the required passes:

```powershell
latexmk -pdf -interaction=nonstopmode -halt-on-error -cd manuscript/main.tex
```

Clean generated files with:

```powershell
latexmk -C -cd manuscript/main.tex
```

When compiling directly with `pdflatex`, remove auxiliary files while retaining the PDF with:

```powershell
Remove-Item manuscript/main.aux, manuscript/main.bbl, manuscript/main.blg, manuscript/main.log, manuscript/main.out -ErrorAction SilentlyContinue
```
