---
id: 005
title: fix-mobile-logout-hidden
stage: misc
date: 2025-12-28
surface: agent
model: gemini
feature: 017-user-auth-profile
branch: main
user: Mutahir
command: replace
labels: ["css", "bugfix", "mobile"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - Textbook/src/css/custom.css
tests:
 - manual verification
---

## Prompt

cant see the button on the mobile screen .. ? fix thi s

## Response snapshot

The CSS rule `.navbar__items--right > .navbar__item:not(...)` was aggressively hiding all items in the mobile top navbar except for Sign In/Sign Up. I updated this rule to also exclude `.mobile-only-nav-item`, allowing the Logout button to be displayed.

## Outcome

- ✅ Impact: Logout button should now be visible on mobile screens.
- 🧪 Tests: Manual verification of CSS selector logic.
- 📁 Files: Modified custom.css.
- 🔁 Next prompts: None immediate.
- 🧠 Reflection: CSS specificities and blanket "hide everything except X" rules can easily cause regressions when new items are added.

## Evaluation notes (flywheel)

- Failure modes observed: unexpected side effect of previous CSS cleanup.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
