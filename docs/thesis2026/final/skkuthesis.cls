%%=============================================================================
%% skkuthesis.cls
%% 성균관대학교 박사학위논문(영문) LaTeX 클래스
%% - 원본 Word 서식("박사 논문 영문 서식")을 기준으로 작성
%% - XeLaTeX 또는 LuaLaTeX 로 컴파일하세요 (한글 처리를 위해 kotex 사용)
%%
%% [핵심 사양]
%%   용지       : A4
%%   여백       : 상/하 53mm, 좌/우 30mm, 머리말/꼬리말 15mm
%%   본문       : 명조(신명조 계열) 11pt, 줄간격(약 200% / 페이지당 25행), 양쪽 정렬
%%   장(Chapter): 16pt 굵게 가운데  ("Chapter N. TITLE")
%%   절(Section): 14pt 굵게 왼쪽    ("N. ...")
%%   항(Subsec) : 12pt 굵게 왼쪽    ("A. ...")
%%   각주       : 9pt
%%   쪽번호     : 앞부분(목차·초록)은 소문자 로마숫자(i, ii ...), 본문은 1, 2, 3 ...
%%=============================================================================
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{skkuthesis}[2026/06/05 v1.0 SKKU PhD Dissertation (English)]

%% ---- 엔진 확인 (XeLaTeX / LuaLaTeX 필요) -----------------------------------
\RequirePackage{iftex}
\ifXeTeX\else\ifLuaTeX\else
  \ClassError{skkuthesis}%
    {This class requires XeLaTeX or LuaLaTeX}%
    {한글 처리를 위해 'xelatex main.tex' 또는 'lualatex main.tex' 로 컴파일하세요.}%
\fi\fi

%% ---- report 기반, 옵션 전달 ------------------------------------------------
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{report}}
\ProcessOptions\relax
\LoadClass[11pt,a4paper,oneside,openany]{report}

%%=============================================================================
%% 패키지
%%=============================================================================
% 한글: kotex/xetexko 대신 fontspec 으로 처리(영문 본문 + 국문 표지/논문요약).
% NanumMyeongjo .ttf 가 한글 글리프를 모두 포함하므로 \setmainfont 만으로 한글이 렌더링됨.
\RequirePackage{fontspec}
\RequirePackage{geometry}
\RequirePackage{setspace}
\RequirePackage{graphicx}
\RequirePackage{array}
\RequirePackage{booktabs}
\RequirePackage{threeparttable}
\RequirePackage{float}        % 표/그림 위치 고정: [H]
\RequirePackage{multirow}     % 셀 세로 병합: \multirow
\RequirePackage{tabularx}     % 너비 자동 조절 표: tabularx, 컬럼 X
\RequirePackage{algorithm}    % 알고리즘 플로트(캡션 "Algorithm N.N")
\RequirePackage{algpseudocode}% 의사코드 본문(\State, \For, \If ...)
\RequirePackage{amsmath}
\RequirePackage{amssymb}
\RequirePackage{xcolor}
\RequirePackage{enumitem}
\RequirePackage[numbers,sort&compress]{natbib}  % 번호식 인용 [1], [2-4]
\RequirePackage[explicit]{titlesec}
\RequirePackage{tocloft}
\RequirePackage{indentfirst}      % 제목 바로 뒤 첫 문단도 들여쓰기
\RequirePackage[font=normalsize,labelfont=bf,labelsep=period,%
                justification=raggedright,singlelinecheck=false]{caption}
\RequirePackage{rotating}         % 측면 표지(책등) 회전용
\RequirePackage[hidelinks]{hyperref}
\RequirePackage{chngcntr}

%%=============================================================================
%% 글꼴 — 프로젝트에 동봉된 나눔글꼴 파일(./fonts/)을 직접 사용.
%%   시스템 글꼴 설치가 필요 없으므로 로컬·Overleaf 어디서나 동일하게 컴파일됩니다.
%%   (반드시 XeLaTeX/LuaLaTeX. Overleaf: Menu → Compiler → XeLaTeX)
%%   나눔글꼴 = SIL Open Font License (프로젝트 동봉·재배포 가능)
%%   ※ 자간(LetterSpace)은 폰트 크기의 % 단위. 0=기본, 숫자가 클수록 넓어짐.
%%   ※ 다른 글꼴을 쓰려면 fonts/ 에 .ttf 를 넣고 아래 이름만 바꾸세요.
%%=============================================================================
\setmainfont{NanumMyeongjo}[
  Path = fonts/, Extension = .ttf, UprightFont = *, BoldFont = *Bold,
  AutoFakeSlant = 0.2, LetterSpace = 2.0]
\setsansfont{NanumGothic}[
  Path = fonts/, Extension = .ttf, UprightFont = *, BoldFont = *Bold]
\setmonofont{NanumGothicCoding}[
  Path = fonts/, Extension = .ttf, UprightFont = *, BoldFont = *Bold]

% --- 한글 전용 폰트 명령(원래 kotex/xetexko 제공)은 fontspec 전용 환경에서 불필요.
%     \setmainfont 의 NanumMyeongjo 가 한글까지 모두 커버하므로 아래는 무시(no-op) 처리. ---
\providecommand{\setmainhangulfont}[2][]{}
\providecommand{\setsanshangulfont}[2][]{}
\providecommand{\setmonohangulfont}[2][]{}
\providecommand{\sethangulfont}[2][]{}

% --- 라틴 확장 글리프(예: ü, é)용 폴백 폰트: NanumMyeongjo 에 없는 악센트 문자를 처리 ---
\newfontfamily\latinfont{TeX Gyre Termes}

% --- 줄 넘침(overfull \hbox) 완화: 긴 \texttt 식별자/URL 이 많은 문서에서 안전하게 작동 ---
\setlength{\emergencystretch}{3em}   % 마지막 수단으로 행간 여유를 두어 넘침 줄임
\hbadness=2500                       % 사소한 underfull 경고 억제
\tolerance=2000                      % 줄바꿈 허용폭 확대
\sloppy                              % URL/긴 토큰이 많은 학위논문에 적합

%%=============================================================================
%% 페이지 레이아웃
%%=============================================================================
\geometry{
  a4paper,
  top      = 53mm,
  bottom   = 53mm,
  left     = 30mm,
  right    = 30mm,
  headsep  = 8mm,
  footskip = 38mm,   % 꼬리말(쪽번호)이 아래 끝에서 15mm 위에 오도록
}

%% 본문 줄간격(setspace의 \setstretch).
%%   목표: 행간 약 20.9pt (= 사용자 설정 \fontsize{11}{11}+\setstretch{1.9} 와 동일).
%%   클래스 기본 행간(13.6pt)에 대해 20.9/13.6 ≈ 1.537 로 맞춤(제목 뒤에도 일정하게 유지됨).
%%   더 넓게/좁게: 이 값만 조정. (1.46≈18pt / 1.54≈20.9pt / 1.69≈23pt)
\newcommand{\thesisbodystretch}{1.537}

%% 목차/표목차/그림목차/알고리즘목차 항목의 줄간격(기본=본문과 동일).
%%   목차만 더 촘촘하게 하려면 값을 낮추세요(예: 1.2). 단행간은 1.
\newcommand{\thesistocstretch}{\thesisbodystretch}

%% 캡션(표/그림 설명)도 본문과 같은 줄간격으로(요청 사항). 자간은 글꼴에 있어 자동 적용.
%%   ※ caption 패키지는 기본적으로 캡션을 단행간으로 두는데(일반적 관례),
%%      여러 줄 캡션을 본문처럼 넓히려고 stretch 를 본문 값과 동일하게 맞춤.
%%      관례대로 캡션만 좁게 두려면 아래 줄을 지우거나 stretch=1 로 바꾸세요.
\captionsetup{font+={stretch=\thesisbodystretch}}
\captionsetup[table]{justification=centering}
\captionsetup[figure]{justification=raggedright}

%% 첫줄 들여쓰기 1.5em, 문단 사이 추가 간격 없음
\setlength{\parindent}{1.5em}
\setlength{\parskip}{0pt}
\raggedbottom

%% 쪽번호: plain 페이지 스타일(아래 가운데). 머리말 없음.
\pagestyle{plain}
\setlength{\headheight}{15pt}

%%=============================================================================
%% 번호 체계 (원본 본문 기준: 장 → 절 N. → 항 A. → 목 1))
%%   ※ 일반적인 'N.M'(장.절) 방식을 원하면 아래 "대안" 블록 주석을 참고하세요.
%%=============================================================================
% ---- 학회 표준 'N.M / N.M.K' 번호 (원본 Word 문서와 동일: 3.1 / 3.1.1) ----
\renewcommand{\thesection}{\thechapter.\arabic{section}}
\renewcommand{\thesubsection}{\thesection.\arabic{subsection}}
\renewcommand{\thesubsubsection}{\thesubsection.\arabic{subsubsection}}

% 목(subsubsection)까지 번호를 매김, 목차에는 절·항까지만 표시.
\setcounter{secnumdepth}{3}
\setcounter{tocdepth}{2}

%% 그림/표/수식/알고리즘 번호를 장 단위로 ("3.1") — 원본 Word 문서와 동일
\counterwithin{table}{chapter}
\counterwithin{figure}{chapter}
\counterwithin{equation}{chapter}
\counterwithin{algorithm}{chapter}

%%=============================================================================
%% 제목(장/절/항) 서식  -- titlesec
%%=============================================================================
% 장(Chapter): "Chapter N. TITLE" 16pt 굵게 가운데, 새 페이지에서 시작
\titleformat{\chapter}[block]
  {\centering\bfseries\fontsize{16}{20}\selectfont}
  {Chapter \thechapter.\ }{0pt}{#1}
\titleformat{name=\chapter,numberless}[block]
  {\centering\bfseries\fontsize{16}{20}\selectfont}
  {}{0pt}{#1}
\titlespacing*{\chapter}{0pt}{0pt}{2\baselineskip}

% 절(Section): "N. ..." 14pt 굵게 왼쪽
\titleformat{\section}[block]
  {\bfseries\fontsize{14}{18}\selectfont}
  {\thesection.\ }{0pt}{#1}
\titlespacing{\section}{0pt}{1.6\baselineskip}{0.8\baselineskip}

% 항(Subsection): "A. ..." 12pt 굵게 왼쪽
\titleformat{\subsection}[block]
  {\bfseries\fontsize{12}{15}\selectfont}
  {\thesubsection.\ }{0pt}{#1}
\titlespacing{\subsection}{0pt}{1.2\baselineskip}{0.5\baselineskip}

% 목(Subsubsection): "1) ..." 11pt 굵게 왼쪽
\titleformat{\subsubsection}[block]
  {\bfseries\normalsize}
  {\thesubsubsection)\ }{0pt}{#1}
\titlespacing{\subsubsection}{0pt}{1\baselineskip}{0.4\baselineskip}

%%=============================================================================
%% 목차 / 표목차 / 그림목차  -- tocloft
%%   원본처럼 가운데 16pt 굵은 제목 + 점선(leader) + 오른쪽 쪽번호로,
%%   세 목록이 페이지를 강제로 넘기지 않고 이어지도록 구성.
%%=============================================================================
% 목록 제목 공통 출력 (초록 / References 등 tocloft 밖에서 사용)
\newcommand{\thesis@listheading}[1]{%
  \par\addvspace{1.5\baselineskip}%
  {\centering\bfseries\fontsize{16}{20}\selectfont #1\par}%
  \addvspace{1.2\baselineskip}%
}

% 목차/표목차/그림목차 제목: 가운데 16pt 굵게.
%   tocloft 이 런타임에 \tableofcontents 등을 자기 버전으로 덮어쓰므로,
%   \AtBeginDocument 에서 다시 우리 버전으로 정의해 '항상' 우리가 이긴다.
%   제목은 \thesis@listheading(=LaTeX \centering, 버전 무관 확실한 가운데정렬)로,
%   항목은 tocloft 의 \l@chapter 등(점선+쪽번호)으로 그대로 출력 → 두 마리 토끼.
\renewcommand{\contentsname}{Table of Contents}
\renewcommand{\listtablename}{List of Tables}
\renewcommand{\listfigurename}{List of Figures}
\renewcommand{\listalgorithmname}{List of Algorithms}

\AtBeginDocument{%
  \newcommand{\thesis@flowlist}[2]{% #1=제목, #2=toc/lot/lof
    \thesis@listheading{#1}%
    \begingroup
      \parindent\z@ \parskip\z@skip \setstretch{\thesistocstretch}%
      \@starttoc{#2}%
    \endgroup
  }%
  \renewcommand{\tableofcontents}{\thesis@flowlist{\contentsname}{toc}}%
  \renewcommand{\listoftables}{\thesis@flowlist{\listtablename}{lot}}%
  \renewcommand{\listoffigures}{\thesis@flowlist{\listfigurename}{lof}}%
  \renewcommand{\listofalgorithms}{%
    \thesis@listheading{\listalgorithmname}%
    \begingroup
      \parindent\z@ \parskip\z@skip \setstretch{\thesistocstretch}%
      \def\l@algorithm{\@dottedtocline{1}{1.5em}{3.2em}}% "2.1  제목 ... 쪽"
      \def\@dotsep{1.5}% 점선 간격을 다른 목록과 동일하게
      \@starttoc{loa}%
    \endgroup
  }%
}

% --- 목차 항목 모양 ---
\renewcommand{\cftdotsep}{1.5}                       % 점선 간격(촘촘하게)
% 장: "N. 제목 .......... 쪽"  (굵지 않게)
\renewcommand{\cftchappresnum}{}
\renewcommand{\cftchapaftersnum}{.}
\renewcommand{\cftchapaftersnumb}{\hspace{0.5em}}
\setlength{\cftchapnumwidth}{2.4em}
\renewcommand{\cftchapfont}{\normalfont\normalsize}
\renewcommand{\cftchappagefont}{\normalfont\normalsize}
\renewcommand{\cftchapleader}{\cftdotfill{\cftdotsep}}  % 장에도 점선
\setlength{\cftbeforechapskip}{2pt}
% 절
\setlength{\cftsecindent}{1.8em}
\setlength{\cftsecnumwidth}{2.6em}
\renewcommand{\cftsecaftersnum}{.}
% 항
\setlength{\cftsubsecindent}{3.6em}
\setlength{\cftsubsecnumwidth}{2.8em}
\renewcommand{\cftsubsecaftersnum}{.}
% 표목차/그림목차: "Table 1.1 ... 쪽" / "Fig. 1.1 ... 쪽"
\renewcommand{\cfttabpresnum}{Table\ }
\renewcommand{\cfttabaftersnum}{}
\setlength{\cfttabnumwidth}{4.6em}
\renewcommand{\cfttabfont}{\normalfont\normalsize}
\renewcommand{\cftfigpresnum}{Fig.\ }
\renewcommand{\cftfigaftersnum}{}
\setlength{\cftfignumwidth}{4.0em}
\renewcommand{\cftfigfont}{\normalfont\normalsize}

%%=============================================================================
%% 메타데이터(논문 정보) 설정 명령
%%=============================================================================
\newcommand{\dissertationtitle}[1]{\def\thesis@title{#1}}
\newcommand{\koreantitle}[1]{\def\thesis@ktitle{#1}}
\newcommand{\authorname}[1]{\def\thesis@author{#1}}
\newcommand{\koreanauthorname}[1]{\def\thesis@kauthor{#1}}
\newcommand{\departmentname}[1]{\def\thesis@dept{#1}}
\newcommand{\koreandepartment}[1]{\def\thesis@kdept{#1}}
\newcommand{\degreedate}[1]{\def\thesis@subdate{#1}}     % 표지/심사청구서 (예: April 2026)
\newcommand{\approvaldate}[1]{\def\thesis@appdate{#1}}    % 인정서        (예: June 2026)
\newcommand{\advisorname}[1]{\def\thesis@advisor{#1}}
\newcommand{\spineyear}[1]{\def\thesis@spineyear{#1}}     % 측면 표지 연도
% 심사위원 성명(서명란 아래에 인쇄). 미입력 시 "Name".
\newcommand{\committeechair}[1]{\def\thesis@chair{#1}}
\newcommand{\committeeone}[1]{\def\thesis@mone{#1}}
\newcommand{\committeetwo}[1]{\def\thesis@mtwo{#1}}
\newcommand{\committeethree}[1]{\def\thesis@mthree{#1}}

% 기본값
\dissertationtitle{Dissertation Title}
\koreantitle{논문 제목}
\authorname{Gilsan Jang}
\koreanauthorname{장길산}
\departmentname{$\bigcirc\bigcirc\bigcirc$ (Dept.\ Name)}
\koreandepartment{OO학과}
\degreedate{April 2026}
\approvaldate{June 2026}
\advisorname{$\bigcirc\bigcirc\bigcirc$ (Major Advisor)}
\spineyear{2026}
\committeechair{Name}
\committeeone{Name}
\committeetwo{Name}
\committeethree{Name}

% PDF 메타데이터
\AtBeginDocument{%
  \hypersetup{pdftitle={\thesis@title}, pdfauthor={\thesis@author}}%
}
% 제목의 줄바꿈(\\)이 PDF 메타데이터 문자열에서는 공백으로 처리되도록(경고 방지)
\pdfstringdefDisableCommands{\def\\{ }}

%%=============================================================================
%% 특수 페이지 (표지/내표지/심사청구서/인정서/국문초록/측면표지)
%%=============================================================================
\newcommand{\thesis@university}{Sungkyunkwan University}

%% ---- 표지(공통 본체): 표지 / 내표지 ----------------------------------------
\newcommand{\thesis@coverbody}{%
  \thispagestyle{empty}%
  \begin{center}\singlespacing
    \vspace*{12mm}
    {\fontsize{14}{18}\selectfont Ph.D.\ Dissertation}\par
    \vspace{14mm}
    {\bfseries\fontsize{22}{28}\selectfont \thesis@title\par}
    \vfill
    {\fontsize{16}{20}\selectfont \thesis@author\par}
    \vfill
    {\fontsize{16}{20}\selectfont
      Department of \thesis@dept\par
      \vspace{2mm}
      The Graduate School\par
      \vspace{2mm}
      \thesis@university\par}
    \vspace{14mm}
  \end{center}%
  \clearpage
}
\newcommand{\makecover}{\thesis@coverbody}
\newcommand{\makeinsidecover}{\thesis@coverbody}

%% ---- 심사청구서 ------------------------------------------------------------
\newcommand{\makesubmission}{%
  \thispagestyle{empty}%
  \begin{center}\singlespacing
    \vspace*{30mm}
    {\bfseries\fontsize{22}{28}\selectfont \thesis@title\par}
    \vspace{26mm}
    {\fontsize{16}{20}\selectfont \thesis@author\par}
    \vspace{22mm}
    {\fontsize{12}{18}\selectfont
      \begin{minipage}{0.86\textwidth}\centering
        A Ph.D.\ Dissertation Submitted to the Department of
        \thesis@dept\ and the Graduate School of \thesis@university\
        in partial fulfillment of the requirements for the degree of
        Ph.D.\ in Engineering
      \end{minipage}\par}
    \vspace{12mm}
    {\fontsize{14}{18}\selectfont \thesis@subdate\par}
    \vfill
    {\fontsize{14}{18}\selectfont
      Supervised by\par
      \vspace{2mm}
      {\fontsize{16}{20}\selectfont \thesis@advisor\par}
      \vspace{2mm}
      Major Advisor\par}
    \vspace{20mm}
  \end{center}%
  \clearpage
}

%% ---- 인정서(서명란) --------------------------------------------------------
% 서명 블록 1개: #1 = 한글 역할(예: 심사위원장), #2 = 영문 역할, #3 = 성명
\newcommand{\thesis@sigblock}[3]{%
  \par\vspace{1.4\baselineskip}%
  \hspace*{0.30\textwidth}%
  \begin{minipage}{0.60\textwidth}
    {\color{black!45}\normalsize #1 signature}\par
    \vspace{-0.15\baselineskip}
    \rule{\linewidth}{0.5pt}\par
    \vspace{0.1\baselineskip}
    {\bfseries #2: #3}
  \end{minipage}\par
}
\newcommand{\makeapproval}{%
  \thispagestyle{empty}\singlespacing
  \vspace*{18mm}
  {\centering\bfseries\fontsize{14}{20}\selectfont
    This certifies that the Ph.D.\ Dissertation of \thesis@author\ is approved.\par}
  \vspace{4mm}
  \thesis@sigblock{심사위원장}{Committee Chair}{\thesis@chair}
  \thesis@sigblock{심사위원 1}{Committee Member 1}{\thesis@mone}
  \thesis@sigblock{심사위원 2}{Committee Member 2}{\thesis@mtwo}
  \thesis@sigblock{심사위원 3}{Committee Member 3}{\thesis@mthree}
  \thesis@sigblock{지도교수}{Major Advisor}{\thesis@advisor}
  \vfill
  {\centering\fontsize{14}{20}\selectfont
    The Graduate School\par
    \vspace{1mm}
    \thesis@university\par
    \vspace{1mm}
    \thesis@appdate\par}
  \vspace{16mm}
  \clearpage
}

%%=============================================================================
%% 영문 초록 (앞부분, 로마숫자 영역)
%%   사용: \begin{englishabstract} ... \keywords{a, b, c} \end{englishabstract}
%%=============================================================================
\newcommand{\keywords}[1]{%
  \par\vspace{1.2\baselineskip}\noindent
  {\bfseries Keywords:}\quad #1\par}
% 본문과 동일한 줄간격 + 어간(단어 간격)을 적용하는 헬퍼.
%   (자간=LetterSpace 는 글꼴에 걸려 있어 어디서나 자동 적용됨)
\newcommand{\thesis@bodyspacing}{%
  \setstretch{\thesisbodystretch}%
  \setlength{\spaceskip}{0.55em plus 0.15em minus 0.05em}%
  \setlength{\xspaceskip}{0.55em plus 0.15em minus 0.05em}%
}

\newenvironment{englishabstract}{%
  \clearpage
  \phantomsection
  \addcontentsline{toc}{chapter}{Abstract}%
  % "Abstract"(16pt 굵게) — 제목 위 섹션 표제
  \par\addvspace{1.5\baselineskip}%
  {\centering\bfseries\fontsize{16}{20}\selectfont Abstract\par}%
  \addvspace{2.0\baselineskip}%
  % 논문 제목(14pt 굵게) — "Abstract" 아래
  {\centering\bfseries\fontsize{14}{18}\selectfont \thesis@title\par}%
  \addvspace{1.2\baselineskip}%
  \thesis@bodyspacing       % 초록 본문도 본문과 같은 줄간격/어간
  \par
}{%
  \par
}

%%=============================================================================
%% 국문 초록 (논문요약, 본문 뒤)
%%   사용: \begin{koreanabstract} ... \koreankeywords{가, 나, 다} \end{koreanabstract}
%%=============================================================================
\newcommand{\koreankeywords}[1]{%
  \par\vspace{1.2\baselineskip}\noindent
  {\bfseries 주제어:}\quad #1\par}
\newenvironment{koreanabstract}{%
  \clearpage
  \phantomsection
  \addcontentsline{toc}{chapter}{Korean Abstract}%
  {\centering\bfseries\fontsize{16}{22}\selectfont \thesis@ktitle\par}
  \vspace{1\baselineskip}
  {\raggedleft\normalsize
    \thesis@kauthor\par
    \thesis@kdept\par
    \thesis@university\par}
  \vspace{1.4\baselineskip}
  \thesis@bodyspacing       % 국문초록 본문도 본문과 같은 줄간격/어간
  \par
}{%
  \par
}

%%=============================================================================
%% 측면 표지(책등) — 제본 시에만 사용. 온라인 제출 시 이 페이지 삭제.
%%=============================================================================
\newcommand{\makesidecover}{%
  \clearpage\thispagestyle{empty}\singlespacing
  \noindent\small
  {\color{red}(Side Cover form 표지측면 서식)}\par
  (제본시에만 사용하세요.\par
  \hspace{1em}온라인 제출 시 삭제 바랍니다.)\par
  (Use only for binding.\ Please remove\par
  \hspace{1em}this page when submitting online.)\par
  \vspace{-30mm}
  \begin{center}
    \begin{turn}{-90}
      \begin{minipage}{0.85\textheight}\centering
        {\bfseries\large Ph.D.\ Dissertation}\par
        \vfill
        {\bfseries\large \thesis@title}\par
        \vfill
        {\bfseries\large \thesis@spineyear}\par
        \vfill
        {\bfseries\large \thesis@author}\par
      \end{minipage}
    \end{turn}
  \end{center}
  \clearpage
}

%%=============================================================================
%% 본문/앞부분/부록 전환 명령
%%=============================================================================
% 앞부분 시작: 소문자 로마숫자, 단행
\newcommand{\frontmatterstyle}{%
  \clearpage
  \pagenumbering{roman}\setcounter{page}{1}%
  \singlespacing
}
% 본문 시작: 아라비아숫자(1부터), 본문 줄간격 + 단어 간격(어간)
\newcommand{\mainmatterstyle}{%
  \clearpage
  \pagenumbering{arabic}\setcounter{page}{1}%
  \thesis@bodyspacing
}
% 부록 시작: 장 번호를 알파벳(A, B ...)으로
\newcommand{\appendixstyle}{%
  \clearpage
  \appendix
  \titleformat{\chapter}[block]
    {\centering\bfseries\fontsize{16}{20}\selectfont}
    {Appendix \thechapter.\ }{0pt}{##1}%
  \thesis@bodyspacing
}

%% References 장(번호 없음, 가운데 16pt) — 본문 어디서든 호출
\newcommand{\makereferencesheading}{%
  \clearpage
  \phantomsection
  \addcontentsline{toc}{chapter}{References}%
  \thesis@listheading{References}%
}

%% natbib 의 참고문헌 제목을 위 양식(가운데 16pt + 목차 등록)과 동일하게.
%%   → main.tex 에서는 \bibliography{...} 만 호출하면 제목이 자동 생성됩니다.
\renewcommand{\bibname}{References}
\AtBeginDocument{%
  \renewcommand{\bibsection}{%
    \clearpage
    \phantomsection
    \addcontentsline{toc}{chapter}{References}%
    \thesis@listheading{References}%
  }%
}

\endinput