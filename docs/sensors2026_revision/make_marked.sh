#!/usr/bin/env bash
# Build the tracked-changes PDF: CFONT (blue sans additions) + red strikethrough
# deletions. Two deletion spans contain \cite/math that break ulem's \sout, so
# they are swapped to a non-strike variant (\DIFdelNS) after latexdiff runs.
set -e
cd "$(dirname "$0")"

docker run --rm -v "$PWD":/work -w /work texlive/texlive:latest bash -c \
  "latexdiff --flatten -t CFONT sensors-4464625-submitted.tex sensors-4464625-r1.tex > sensors-4464625-r1-marked.tex 2>/dev/null; chown $(id -u):$(id -g) sensors-4464625-r1-marked.tex"

python3 - <<'EOF'
p='sensors-4464625-r1-marked.tex'
s=open(p).read()
anchor=r"\providecommand{\DIFdel}[1]{{\protect\color{red} \scriptsize #1}} %DIF PREAMBLE"
assert s.count(anchor)==1
s=s.replace(anchor,
  r"\RequirePackage[normalem]{ulem} %DIF PREAMBLE" "\n"
  r"\providecommand{\DIFdel}[1]{{\protect\color{red}\protect\scriptsize\protect\sout{#1}}} %DIF PREAMBLE" "\n"
  r"\providecommand{\DIFdelNS}[1]{{\protect\color{red}\protect\scriptsize #1}} %DIF PREAMBLE")
# fragile spans: strike-out breaks on \cite / math here -> color-only marking
for frag in [
  "\\DIFdel{, all of which require either stationary acquisition or a dwell\ntime at each viewpoint~\\cite{stopandgo-review,gas-coverage,robotic-ndt}}",
]:
    if s.count(frag)==1:
        s=s.replace(frag, frag.replace("\\DIFdel{","\\DIFdelNS{",1))
frag2="""\\DIFdelbeginFL \\DIFdelFL{:
$A_{\\min}$ is the operative coverage-vs-scans knob}\\DIFdelendFL , \\DIFdelbeginFL \\DIFdelFL{with the }\\DIFdelendFL knee \\DIFdelbeginFL \\DIFdelFL{at
$5$~m\\textsuperscript{2}}\\DIFdelendFL"""
if s.count(frag2)==1:
    s=s.replace(frag2, frag2.replace("\\DIFdelFL{","\\DIFdelNS{"))
open(p,'w').write(s)
EOF

docker run --rm -v "$PWD":/work -w /work texlive/texlive:latest bash -c \
  "pdflatex -interaction=nonstopmode sensors-4464625-r1-marked.tex > m.log 2>&1; pdflatex -interaction=nonstopmode sensors-4464625-r1-marked.tex > m.log 2>&1; grep -c '^!' m.log; tail -1 m.log"
