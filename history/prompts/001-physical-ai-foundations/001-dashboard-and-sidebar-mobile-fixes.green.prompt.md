---
id: 001-dashboard-and-sidebar-mobile-fixes
title: Dashboard and Sidebar Mobile Fixes
stage: green
date: 2025-12-28
surface: agent
model: gemini-2.0-flash-exp
feature: 001-dashboard-ui
branch: main
user: Mutahir
command: fix dashboard mobile layout and sidebar
labels: [css, responsive, mobile, dashboard, sidebar, docusaurus]
links: null
files_yaml:
  - Textbook/src/css/custom.css
  - Textbook/src/components/AppLayout/AppLayout.css
tests_yaml: []
prompt_text: |
  on the dashboard (Responsive screen sizes mobile.tablet) the card it self are not proeprly reposive they are cutting from right after 480 screen size look into it and fix the cards 
  
  secondly (for mobile/Tablet screen size)
     Add Curriculum (Textbook) Sidebar on Mobile/tablet:
         * Fix the issue where the sidebar navigation tree (Parts & Chapters) are  hidden (or not made) in the mobile/tablet burger menu.
         * explicitl  display of .menu__list and .menu items within the .navbar-sidebar, ensuring that the collapsible tree structure is visible and interactive.
         * The "Curriculum" link in the burger menu should correctly display the full hierarchy of the textbook, allowing users to switch chapters easily on mobile/tablet screen devices.
response_text: |
  Fixed dashboard card cutoff on small screens by adding `box-sizing: border-box`, `min-width: 0`, and reducing padding in `custom.css` and `AppLayout.css`.
  Fixed invisible curriculum sidebar on mobile by strictly forcing `display: block` on `.menu` and `.menu__list` classes within `.navbar-sidebar` in `custom.css`.
  
  UPDATE: Reverted aggressive `display: flex !important` and `display: block !important` overrides on `.navbar-sidebar__items` and `.menu` in `custom.css`. This restores the standard Docusaurus behavior where the secondary menu (Curriculum tree) is toggled correctly instead of being conflictingly displayed or hidden. Ensured container visibility and styling are preserved.
  
  FINAL FIX: Removed `display: block !important` from `.navbar-sidebar__item` and `.navbar-sidebar__back` in `custom.css`. These rules were forcing the Primary Menu to overlay or persist when the Secondary Menu (Tree) was active, effectively hiding the chapters. Removal restores correct toggling.

  UPDATE 2: Removed layout overrides (`top`, `height`, `padding`, `display: flex`) from `.navbar-sidebar` and `.navbar-sidebar__items` in `custom.css`... which caused a visual regression.
  
  RESTORED: Re-applied the layout styles (`top: 0`, `height: 100vh`, `padding-top: 5rem`) to `.navbar-sidebar` to fix the broken UI, while maintaining the logic fixes (no forced display on items) to ensure chapter visibility.
  
  CONFIRMED: Verified all fixes are present and added explicit `display: block` to `.menu__list` for robustness.
outcome: success
---