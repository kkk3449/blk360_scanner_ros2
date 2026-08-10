#!/usr/bin/env python3
"""Combine the three per-reviewer response .md files into one XeLaTeX PDF."""
import re

FILES = ["response_reviewer1.md", "response_reviewer2.md", "response_reviewer3.md"]
OUT = "response_to_reviewers_combined.tex"

def esc(t):
    for a, b in [("\\", r"\textbackslash{}"), ("&", r"\&"), ("%", r"\%"),
                 ("$", r"\$"), ("#", r"\#"), ("_", r"\_"), ("~", r"\textasciitilde{}"),
                 ("^", r"\textasciicircum{}")]:
        t = t.replace(a, b)
    return t

def inline(t):
    t = esc(t)
    t = re.sub(r"\*\*(.+?)\*\*", r"\\textbf{\1}", t)
    t = re.sub(r"(?<!\*)\*(?!\*)(.+?)(?<!\*)\*(?!\*)", r"\\emph{\1}", t)
    t = re.sub(r"`(.+?)`", r"\\texttt{\1}", t)
    return t

def md_to_tex(lines):
    out, mode = [], None   # mode: None | 'quote' | 'itemize' | 'enum'
    def close():
        nonlocal mode
        if mode == "quote":
            out.append(r"\end{quoting}")
        elif mode == "itemize":
            out.append(r"\end{itemize}")
        elif mode == "enum":
            out.append(r"\end{enumerate}")
        mode = None
    for ln in lines:
        s = ln.rstrip("\n")
        if s.startswith("### "):
            close(); out.append(r"\subsection*{%s}" % inline(s[4:]))
        elif s.startswith("## "):
            close(); out.append(r"\section*{%s}" % inline(s[3:]))
        elif s.startswith("# "):
            close(); out.append(r"\begin{center}\LARGE\bfseries %s\end{center}" % inline(s[2:]))
        elif s.strip() == "---":
            close(); out.append(r"\medskip\hrule\medskip")
        elif s.startswith("> "):
            if mode != "quote":
                close(); out.append(r"\begin{quoting}"); mode = "quote"
            out.append(inline(s[2:]))
        elif re.match(r"^- ", s):
            if mode != "itemize":
                close(); out.append(r"\begin{itemize}"); mode = "itemize"
            out.append(r"\item " + inline(s[2:]))
        elif re.match(r"^\d+\. ", s):
            if mode != "enum":
                close(); out.append(r"\begin{enumerate}"); mode = "enum"
            out.append(r"\item " + inline(re.sub(r"^\d+\. ", "", s)))
        elif s.strip() == "":
            if mode in ("itemize", "enum", "quote"):
                out.append("")
            else:
                close(); out.append("")
        else:
            if mode in ("itemize", "enum"):   # continuation line of a list item
                out.append(inline(s.strip()))
            else:
                if mode == "quote":
                    close()
                out.append(inline(s))
    close()
    return "\n".join(out)

body = []
for i, f in enumerate(FILES):
    if i:
        body.append(r"\clearpage")
    body.append(md_to_tex(open(f).readlines()))

doc = r"""\documentclass[11pt]{article}
\usepackage[a4paper,margin=2.4cm]{geometry}
\usepackage{fontspec}
\setmainfont{DejaVu Serif}
\setmonofont{DejaVu Sans Mono}
\usepackage{quoting}
\quotingsetup{vskip=4pt,leftmargin=1.6em,rightmargin=1.0em,font={itshape,small}}
\usepackage{enumitem}
\setlist{topsep=3pt,itemsep=2pt}
\usepackage{titlesec}
\titleformat{\section}{\large\bfseries}{}{0pt}{}
\titlespacing*{\section}{0pt}{14pt}{5pt}
\titleformat{\subsection}{\normalsize\bfseries}{}{0pt}{}
\usepackage{parskip}
\usepackage[hidelinks]{hyperref}
\begin{document}
%s
\end{document}
""" % "\n".join(body)

open(OUT, "w").write(doc)
print("wrote", OUT)
