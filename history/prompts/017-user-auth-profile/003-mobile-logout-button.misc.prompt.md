---
id: "003"
title: "Mobile Logout Button"
stage: "misc"
date: "2025-12-28"
surface: "agent"
model: "gemini-2.0-flash-exp"
feature: "017-user-auth-profile"
branch: "main"
user: "Mutahir"
command: "add a logout button to the sidebar (burger menu ) of the homepage .. mobile/tablet screens only"
labels:
  - "ui"
  - "auth"
  - "mobile"
links:
  spec: "specs/017-user-auth-profile/spec.md"
  ticket: null
  adr: null
  pr: null
files:
  - "Textbook/docusaurus.config.ts"
  - "Textbook/src/theme/NavbarItem/index.tsx"
  - "Textbook/src/css/custom.css"
tests: []
---

# User Request
add a logout button to the sidebar (burger menu ) of the homepage .. mobile/tablet screens only

# Assistant Response
Added a "Logout" button to the mobile sidebar by configuring a new navbar item restricted to mobile via CSS and updating the NavbarItem wrapper to handle it. Also ensured the desktop UserMenu does not clutter the mobile view.

## Changes
- **Textbook/docusaurus.config.ts**: Added 'Logout' item with `mobile-only-nav-item` class.
- **Textbook/src/theme/NavbarItem/index.tsx**: Implemented logic for 'Logout' item to render and handle sign out. Added `desktop-only` class to UserMenu.
- **Textbook/src/css/custom.css**: Added `.desktop-only` utility class.

## Outcomes
- Logout button appears in the mobile burger menu when logged in.
- Logout button is hidden on desktop (managed via UserMenu).
- UserMenu (avatar) is hidden on mobile.