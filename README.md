# AED-kit-delivery-robot

## 프로젝트 문서 목적

이 레포는 코드관리를 더 유용하게 하기 위한 목적으로 사용합니다.

## 관리 방식

저희 팀은 크게 localization, perception, planning, control이 있습니다. 그래서 각 팀별로 코드를 관리할 수 있고, 전체적인 코드를 확인할 수 도 있게 세팅했습니다.

### 브랜치

-   만약 깃, 깃허브가 익숙하지 않다면

[깃 & 깃허브](https://codingapple.com/course/git-and-github/)

-   브랜치가 무엇이고, 어떤 용도로 사용하는지

[브랜치](https://codingapple.com/unit/git-3-branch-and-switch/)

저희 팀의 브랜치는 크게 6개입니다.

-   main
-   develop
-   localization
-   perception
-   planning
-   control

### main 브랜치

-   저희가 실제 주행(시범주행) 할 때 사용할 브랜치입니다.
-   이 브랜치는 개발(develop) 브랜치에서 푸시받습니다.

### develop 브랜치

-   저희 각 팀이 작성하고 테스트한 코드를 합치고, 시범 주행을 위한 브랜치입니다.
-   이 브랜치는 localization, perception, planning, control에서 푸시받습니다.

### localization, perception, planning, control

-   각 팀이 실제로 작업할 브랜치입니다.
-   각 팀에서 커밋하고, 푸시를 할 브랜치입니다.

### 브랜치 전략(컨벤션)

[참고](https://inpa.tistory.com/entry/GIT-%E2%9A%A1%EF%B8%8F-github-flow-git-flow-%F0%9F%93%88-%EB%B8%8C%EB%9E%9C%EC%B9%98-%EC%A0%84%EB%9E%B5)

-   오프라인 설명 예정

## 커밋 메세지 컨벤션

```
feat: 코드에 새로운 기능(feature) 추가
fix: 버그 수정
BREAKING CHANGE: 이전 버젼과 호환되지 않는 수정내역으로, !로 표시가능(예: feat!)
docs: 개발 문서 변경
style: 들여쓰기, 따옴표, 세미콜론 등 코드 형식 및 스타일 변경
refactor: 중복된 코드 제거, 변수명 변경, 코드 단순화 등 리팩토링 변경
test: 테스트 관련 코드 변경
build: 빌드 시스템 관련 코드 변경
perf: 성능 개선 관련 코드 변경
chore: 기타 코드 변경
```

### 커밋 메세지 예시

```
feat: Add new feature for localization
- Added feature X for improved localization accuracy
- Updated function Y for better performance
```

## 주의사항

-   논의가 되지 않은 브랜치간 pull request는 금지합니다.
-   변경 사항을 커밋하기 전에 코드를 테스트합니다.
-   빌드 실패 시에는 해당 이슈를 신속하게 수정하고 다시 테스트합니다.

## 이슈 관리 방식

### 이슈 만들기

[노션 링크 확인](https://www.notion.so/outlu/e0193f5d19df4121927edb927d961d94?pvs=4#962b6db6d7ef47f9beab7d9d03cee797)

### 이슈와 커밋

[노션 링크 확인](https://www.notion.so/outlu/e0193f5d19df4121927edb927d961d94?pvs=4#5ac883a8bee34938977bf26fead93ab0)

### 이슈와 PR

[노션 링크 확인](https://www.notion.so/outlu/e0193f5d19df4121927edb927d961d94?pvs=4#c16d24a162fe4f95b7c9ae9f5af8ae59)
