---
id: "0008"
title: Fix Dashboard Cards and Mobile Sidebar
stage: refactor
date: 2025-12-28
surface: agent
model: gemini-2.0-flash-exp
feature: general
branch: main
user: Mutahir
command: fix sidebar and dashboard
labels: [css, docusaurus, responsive, mobile]
links: null
files:
 - Textbook/src/css/custom.css
tests: []
---

# Prompt

on the dashboard (Responsive screen sizes mobile.tablet) the card it self are not proeprly reposive they are cutting from right after screen size smaller than 480px look into it and fix the cards 

secondly (for mobile/Tablet screen size)
   On Curriculum (Textbook) Sidebar on Mobile/tablet:
       * Fix the issue where the sidebar navigation tree (Parts & Chapters) are  hidden (or not made) in the mobile/tablet burger menu.
       * explicitl  display of .menu__list and .menu items within the .navbar-sidebar, ensuring that the collapsible tree structure is visible and interactive.
       * The "Curriculum" link in the burger menu should correctly display the full hierarchy of the textbook, allowing users to switch chapters easily on mobile/tablet screen devices.

# Response

I have fixed the requested issues in `Textbook/src/css/custom.css`:

1.  **Dashboard Cards Responsiveness:**
    *   Added a media query for `max-width: 480px`.
    *   Forced `.dashboard-grid .card` to have `width: 100% !important`, `max-width: 100% !important`, and `overflow-x: hidden`.
    *   This ensures cards fit within the container and do not get cut off or cause layout shifts on small screens.

2.  **Curriculum (Textbook) Sidebar Visibility:**
    *   Explicitly added `display: block !important` to `.navbar-sidebar__item`, `.navbar-sidebar .menu`, and `.navbar-sidebar .menu__list`.
    *   This ensures the navigation tree is always rendered and visible within the mobile sidebar, allowing users to navigate the full hierarchy.

No new files were created. Changes were applied directly to the custom CSS.
