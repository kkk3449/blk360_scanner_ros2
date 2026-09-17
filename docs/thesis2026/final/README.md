# 성균관대학교 박사학위논문(영문) LaTeX 양식

성균관대학교 공식 Word 서식("박사 논문 영문 서식")을 LaTeX로 옮긴 템플릿입니다.
표지·내표지·심사청구서·인정서·목차·영문초록·본문·참고문헌·부록·국문초록·측면표지를
모두 포함합니다.

한글 처리를 위해 **반드시 XeLaTeX(또는 LuaLaTeX)** 로 컴파일하세요. pdfLaTeX는 동작하지 않습니다.

---

## 1. 파일 구성

```
.
├── skkuthesis.cls        # 논문 클래스(여백·글꼴·제목·번호·특수페이지 등 모든 서식)
├── main.tex              # 본체. 논문 정보 입력 + 문서 순서 정의
├── references.bib        # 참고문헌 (Zotero/Better BibTeX 내보내기 호환)
├── .latexmkrc            # latexmk가 자동으로 XeLaTeX를 쓰게 하는 설정
├── fonts/                # 동봉된 나눔글꼴(.ttf) — 시스템 설치 불필요
│   ├── NanumMyeongjo.ttf / NanumMyeongjoBold.ttf       (본문 명조)
│   ├── NanumGothic.ttf / NanumGothicBold.ttf           (sans)
│   └── NanumGothicCoding.ttf / ...Bold.ttf             (mono)
├── chapters/
│   ├── chapter1.tex      # 서론(절·항·목 번호 예시)
│   ├── chapter2.tex      # 그림(검정박스)·표·수식·알고리즘·각주·인용 예시
│   ├── conclusion.tex    # 결론
│   └── appendix1.tex     # 부록
└── figures/              
    └── test.png          # 이미지 파일
```

---

## 2. 컴파일 방법

### 권장: latexmk

함께 들어있는 `.latexmkrc` 덕분에 플래그 없이도 **XeLaTeX로 빌드**됩니다.

```bash
latexmk main.tex          # .latexmkrc 가 있으면 자동으로 xelatex 사용
# (또는 명시적으로) latexmk -xelatex main.tex
```

### 수동 컴파일

```bash
xelatex main
bibtex  main
xelatex main
xelatex main
```

참고문헌(인용)을 쓰지 않는다면 `xelatex main` 두 번이면 목차·상호참조까지 완성됩니다.

### VS Code (LaTeX Workshop) 자동 빌드

LaTeX Workshop은 저장 시 자동 빌드가 기본이지만, 기본 엔진이 pdflatex라 이 템플릿에선
실패합니다. 아래 중 하나로 XeLaTeX를 쓰게 하세요.

- **`.latexmkrc` 사용(가장 간단)**: 이 파일을 `main.tex`와 같은 폴더에 두면 끝.
  모든 xelatex 프로젝트에 적용하려면 같은 내용을 `~/.latexmkrc`(홈)에도 두세요.
  (latexmk는 rc 파일을 *홈* 또는 *명령 실행 폴더*에서만 찾고, 상위 폴더는 보지 않습니다.)
- **settings.json 레시피**: `.vscode/settings.json`에 `latexmk -xelatex …` 레시피를
  넣고 기본으로 지정.

> 빌드 로그에 `applying rule 'xelatex'`, `Rc files read:`에 내 `.latexmkrc` 경로가
> 보이면 정상입니다.

### Overleaf

이 프로젝트는 글꼴(`fonts/`)을 동봉하므로 Overleaf에서도 그대로 컴파일됩니다.

1. 프로젝트 전체(`fonts/` 폴더 포함)를 Overleaf에 업로드. `main.tex`와 `fonts/`가
   **같은 폴더(프로젝트 루트)**에 있어야 합니다(글꼴을 `Path = fonts/`로 불러오므로).
2. **Menu → Settings → Compiler 를 `XeLaTeX`로 변경** (이게 핵심 — 기본 pdfLaTeX면 실패).
3. Recompile.

> Overleaf는 엔진을 Menu 설정으로 정하므로, `.latexmkrc`만으로는 부족할 수 있습니다.
> 반드시 Compiler를 XeLaTeX로 바꿔주세요. 글꼴은 동봉돼 있어 별도 설치가 필요 없습니다.

---

## 3. 글꼴 (동봉됨 — 설치 불필요)

본문 명조·sans·mono 모두 **나눔글꼴 `.ttf`를 `fonts/`에 동봉**해 `Path`로 직접
불러옵니다. 따라서 로컬/Overleaf 어디서나 시스템 글꼴 설치 없이 동일하게 나옵니다.

| 용도 | 글꼴(동봉 파일) |
|------|----------------|
| 본문(영문·한글) | NanumMyeongjo (명조) |
| sans | NanumGothic |
| mono | NanumGothicCoding |

나눔글꼴은 SIL Open Font License라 프로젝트에 함께 배포할 수 있습니다.

> 시스템에 설치된 글꼴을 쓰고 싶거나 학교 지정 글꼴(신명조/바탕 등)로 바꾸려면
> 5-(1)을 참고하세요.

한글 처리에는 `kotex`(cjk-ko)이 필요합니다. TeX Live(full)와 Overleaf에는 기본 포함돼
있고, 로컬 Ubuntu에서 없다면 `sudo apt install texlive-lang-korean`로 설치하세요.

---

## 4. 논문 정보 수정

`main.tex` 상단의 메타데이터만 고치면 표지·심사청구서·인정서·초록에 자동 반영됩니다.

```latex
\dissertationtitle{Semantic Task Management Framework for\\
  Heterogeneous Multi-Robot Systems}   % 영문 제목 (줄바꿈은 \\)
\koreantitle{이종 다중 로봇 시스템을 위한\\의미론적 작업 관리 프레임워크}
\authorname{Gilsan Jang}               % 영문 성명
\koreanauthorname{장길산}              % 국문 성명
\departmentname{Electrical and Computer Engineering}
\koreandepartment{전자전기컴퓨터공학과}
\degreedate{April 2026}                % 표지·심사청구서 날짜
\approvaldate{June 2026}               % 인정서(심사 통과) 날짜
\advisorname{Gildong Hong}            % 지도교수
\spineyear{2026}                       % 측면표지 연도
\committeechair{Gildong Hong}         % 심사위원장
\committeeone{Cheolsu Kim}            % 심사위원 1
\committeetwo{Younghee Lee}           % 심사위원 2
\committeethree{Minjun Park}          % 심사위원 3
```

> 8월 졸업이어도 학교 지침상 날짜는 보통 4월/6월로 적습니다(학과 확인 권장).

---

## 5. 자주 바꾸는 설정

### (1) 글꼴 교체 (영문을 Times 계열로 분리 / 학교 지정 글꼴)

기본은 영문·한글 모두 동봉된 **NanumMyeongjo**를 `fonts/`에서 직접 불러옵니다.
`skkuthesis.cls`의 글꼴 블록에서 바꿉니다.

```latex
% (현재 기본) 동봉 파일에서 로드 — 시스템 설치 불필요
\setmainfont{NanumMyeongjo}[Path=fonts/, Extension=.ttf,
  UprightFont=*, BoldFont=*Bold, AutoFakeSlant=0.2, LetterSpace=2.0]
\setmainhangulfont{NanumMyeongjo}[Path=fonts/, Extension=.ttf,
  UprightFont=*, BoldFont=*Bold, AutoFakeSlant=0.2, LetterSpace=2.0]
```

- **다른 .ttf를 동봉해 쓰기**: 그 파일을 `fonts/`에 넣고 위 이름만 바꾸세요
  (예: 학교 지정 신명조 파일).
- **시스템에 설치된 글꼴로 쓰기**(로컬 전용): `Path=fonts/, Extension=.ttf,
  UprightFont=*, BoldFont=*Bold`를 지우고 글꼴 패밀리명만 남기세요. 단 이 경우
  Overleaf 등 다른 환경에서 그 글꼴이 없으면 실패할 수 있습니다.
- **영문만 Times 계열로 분리**: `\setmainfont`만 시스템 글꼴 `{TeX Gyre Termes}`로
  교체(한글 `\setmainhangulfont`는 그대로 둠).

### (2) 번호 체계: 장→절(N.)→항(A.)→목(1) ↔ 학회식 N.M

기본값은 원본 서식 본문과 동일한 **N. / A. / 1)** 방식입니다.
일반적인 **1.1 / 1.1.1** 방식을 원하면 `skkuthesis.cls`의 번호 블록에서
기본 3줄을 주석 처리하고 "대안" 3줄의 주석을 해제하세요.

```latex
% 기본(서식 그대로)
\renewcommand{\thesection}{\arabic{section}}
\renewcommand{\thesubsection}{\Alph{subsection}}
\renewcommand{\thesubsubsection}{\arabic{subsubsection}}

% 대안(학회 표준 1.1 / 1.1.1)
% \renewcommand{\thesection}{\thechapter.\arabic{section}}
% \renewcommand{\thesubsection}{\thesection.\arabic{subsection}}
% \renewcommand{\thesubsubsection}{\thesubsection.\arabic{subsubsection}}
```

### (3) 줄 간격 · 자간 · 어간(단어 간격)

**줄 간격** — `skkuthesis.cls`의 한 줄로 조절합니다. 기본값 1.537은 워드의
`11pt + 줄간격 1.9`(행간 약 20.9pt)와 동일하게 맞춘 값입니다.

```latex
\newcommand{\thesisbodystretch}{1.537}   % 1.46≈18pt / 1.54≈20.9pt / 1.69≈23pt
```

> 클래스 기본 행간(13.6pt)에 stretch를 곱하는 방식이라, `\fontsize{11}{11}`처럼
> 직접 지정하는 것과 달리 **제목 뒤에서도 행간이 일정하게 유지**됩니다.

**자간(글자 사이)** — 글꼴 줄의 `LetterSpace` 숫자(폰트 크기 % 단위, 0=기본).

```latex
\setmainfont{NanumMyeongjo}[Path=fonts/, Extension=.ttf,
  UprightFont=*, BoldFont=*Bold, AutoFakeSlant=0.2, LetterSpace=2.0]       % 영문
\setmainhangulfont{NanumMyeongjo}[Path=fonts/, Extension=.ttf,
  UprightFont=*, BoldFont=*Bold, AutoFakeSlant=0.2, LetterSpace=2.0]       % 한글
```

**어간(단어 사이)** — `\mainmatterstyle`·`\appendixstyle` 안의 `\spaceskip` 값(본문에만 적용).

```latex
\setlength{\spaceskip}{0.55em plus 0.15em minus 0.05em}
\setlength{\xspaceskip}{0.55em plus 0.15em minus 0.05em}
```

**첫 줄 들여쓰기** — 기본 1.5em.

```latex
\setlength{\parindent}{1.5em}
```

### (4) 측면표지(책등) 제거 — 온라인 제출본

제본용 측면표지는 온라인 제출 시 삭제합니다. `main.tex` 마지막 줄을 주석 처리하세요.

```latex
% \makesidecover
```

---

## 6. 참고문헌

- 인용은 `\citep{key}`(괄호형) 또는 `\citet{key}`(서술형)을 사용합니다.
- 기본 스타일은 공학 표준 번호식 `ieeetr`(`[1]`, `[2-4]`)입니다.
  저자-연도식을 원하면 `main.tex`의 `\bibliographystyle{ieeetr}`를
  `plainnat` 등으로 바꾸세요.
- **Zotero + Better BibTeX**: 컬렉션을 `.bib`로 내보내 `references.bib`를 대체하면 됩니다.

### 논문(저널) 게재본을 묶는 학위논문(sandwich thesis)인 경우

장마다 별도 참고문헌을 두고 싶으면 `chapterbib` 또는 `bibunits` 패키지를 추가로
사용할 수 있습니다(현재 템플릿은 끝에 통합 참고문헌 1개 방식). 또한 게재 논문을
장으로 넣을 때는 각 장 첫머리에 출처를 밝히는 문구를 넣는 것이 일반적입니다.
예:

```latex
% 장 시작 부분에:
\footnotetext{The content of this chapter was published in \emph{저널명}, 2026.}
```

---

## 7. 여백/레이아웃 (원본 서식 기준)

- 용지: A4
- 여백: 상·하 53mm, 좌·우 30mm, 머리말·꼬리말 15mm
- 본문: NanumMyeongjo 11pt, 양쪽 정렬, 행간 약 20.9pt(더블), 첫 줄 들여쓰기 1.5em
- 쪽번호: 표지류(쪽번호 없음) → 앞부분(목차·초록) 소문자 로마숫자 i, ii →
  본문 아라비아숫자 1, 2, 3 (모두 아래 가운데)

여백 값은 모두 `skkuthesis.cls`의 `\geometry{...}`에서 조정할 수 있습니다.
학과/대학원 제출 규정에 맞춰 필요 시 수정하세요.

---

## 빌드 노트 (본심사판 v2026-09-15a)

- **참고문헌은 BibTeX** (`references.bib`, 117편, `ieeetr`): `xelatex main ; bibtex main ; xelatex main ; xelatex main`.
  Overleaf는 자동 처리. 로컬은 `docker run --rm -v "$PWD":/work -w /work texlive/texlive:latest sh -c "xelatex main; bibtex main; xelatex main; xelatex main"`.
- 장 파일: `chapters/chapter1..5.tex`, `chapter6.tex`(= `chapter6a.tex` + `chapter6b.tex` 입력), `conclusion.tex`.
- 그림 접두사: `sen_*` Sensors 2026, `ele_*` Electronics, `ictc_*` ICTC 2026, `raa_*` RAAICON 2026, 나머지는 예심판 그림.
- 표지 날짜(`\degreedate`, `\approvaldate`, `\spineyear`)와 심사위원 성명은 `main.tex` 상단에서 확정 후 수정.

## 빌드 노트 (r3_2 — Word→LaTeX 포팅판, 예심판 당시)

- **컴파일러: XeLaTeX 필수** (`latexmk` 사용 시 `.latexmkrc`가 자동으로 XeLaTeX 선택).
- 본문은 **영문**, 표지·논문요약(국문)은 동봉된 **NanumMyeongjo**로 렌더링됩니다.
- 원본 SKKU 템플릿은 `kotex`를 쓰지만, 영문 본문에서는 불필요하고 일부 환경에서 의존성
  (`kolabels-utf.sty`)이 빠질 수 있어 **`fontspec` 단독 방식**으로 변경했습니다.
  `\setmainfont{NanumMyeongjo}`가 한글까지 모두 커버하므로 표지·논문요약 한글이 정상 출력됩니다.
  (Overleaf 등 `kotex`가 있는 환경에서도 그대로 컴파일됩니다.)
- "Güler" 등 NanumMyeongjo에 없는 라틴 악센트 문자는 `\latinfont`(TeX Gyre Termes)로 처리.
- 번호 체계는 Word 원본과 일치하도록 **장 단위**로 설정: 절 `3.1 / 3.1.1`, 그림·표·수식 `3.1`, `(3.4)` 등.
  - 그림은 Word 본문 등장 순서가 캡션 번호와 달라 `\setcounter{figure}{...}`로 번호를 고정.
  - 수식은 `\tag{...}`로 원본의 문자 접미사 번호((3.9d) 등)까지 재현.
  - 참고문헌은 원본 번호 [1]–[31] 유지를 위해 수동 `thebibliography` 사용 (bibtex 미사용).
- **심사위원 이름은 비워둠** — `main.tex` 상단 메타데이터에서 직접 채우세요.

### 빌드 방법
```bash
latexmk -xelatex main.tex     # 권장 (자동 다회 패스)
# 또는
xelatex main.tex && xelatex main.tex && xelatex main.tex
```
