# .latexmkrc — 이 폴더(또는 main.tex가 있는 폴더)에 두면
#   latexmk / VS Code(LaTeX Workshop) 빌드가 자동으로 XeLaTeX를 사용합니다.
#   (kotex + fontspec + 시스템 글꼴 NanumMyeongjo 때문에 XeLaTeX 필수)

$pdf_mode   = 5;   # 5 = xelatex (1=pdflatex, 4=lualatex)
$bibtex_use = 2;   # 참고문헌(bibtex)이 필요하면 자동 실행, .bbl은 생성물로 처리

# xelatex 실행 옵션 (synctex: VS Code 정방향/역방향 클릭 이동 지원)
$xelatex = 'xelatex -synctex=1 -interaction=nonstopmode -file-line-error %O %S';
