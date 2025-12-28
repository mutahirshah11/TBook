---
id: "0011"
title: Custom Navbar Items Implementation
stage: feature
date: 2025-12-28
surface: agent
model: gemini-2.0-flash-exp
feature: general
branch: main
user: Mutahir
command: implement custom navbar items
labels: [docusaurus, react, navbar, mobile]
links: null
files:
 - Textbook/src/theme/NavbarItem/index.tsx
 - Textbook/src/theme/NavbarItem/MobileLogoutButton.tsx
 - Textbook/src/theme/NavbarItem/ChapterDropdown.tsx
 - Textbook/src/components/AppLayout/AppSidebarMobile.tsx
 - Textbook/docusaurus.config.ts
 - Textbook/src/css/custom.css
tests: []
---

# Prompt

In mobile screen/tablet screen sizez : 
1. in homepage add a logout button when the user is logged in to the burger menu so user can logout. 
2. in the curriculum burger menu make it same as the homepage , and at the right side of the navbar outside in the page add a drop down box which has all chapter from 1 till end and user can click any of them and switch to it . 
3. in the dashboard (navbar not the burger menu) make the logo and name as same as other as they are in other pages .

Then fixed a crash due to unhandled custom navbar item types.

# Response

I implemented the requested features:

1.  **Mobile Logout Button:**
    *   Created `Textbook/src/theme/NavbarItem/MobileLogoutButton.tsx`.
    *   Added to `docusaurus.config.ts`.
    *   Configured to only show on mobile (`display: none` on desktop via CSS).
    *   Shows "Sign Out" button only when logged in.

2.  **Chapter Dropdown & Curriculum Menu:**
    *   Created `Textbook/src/theme/NavbarItem/ChapterDropdown.tsx` with a `<select>` element populated with chapters.
    *   Added to `docusaurus.config.ts` in `position: 'right'`.
    *   Modified `Textbook/src/css/custom.css` to hide the default docs sidebar tree in the mobile menu (`.theme-doc-sidebar-menu`), making the menu cleaner ("same as homepage").
    *   Ensured the dropdown is visible on the mobile navbar.

3.  **Dashboard Logo:**
    *   Updated `Textbook/src/components/AppLayout/AppSidebarMobile.tsx` to use the correct `img/logo.svg` to match the site branding.

4.  **Crash Fix:**
    *   Updated `Textbook/src/theme/NavbarItem/index.tsx` to explicitly handle and render `custom-MobileLogoutButton` and `custom-ChapterDropdown` types, as swizzling the wrapper bypassed Docusaurus's default resolution.

The site should now function correctly without errors, with the new navigation elements in place.
