---
id: "0010"
title: Fix Dashboard Mobile Cutting Final
stage: refactor
date: 2025-12-28
surface: agent
model: gemini-2.0-flash-exp
feature: general
branch: main
user: Mutahir
command: make all cards proper
labels: [css, responsive, dashboard, mobile, hotfix]
links: null
files:
 - Textbook/src/css/custom.css
tests: []
---

# Prompt

make all cards proper !!! they are not !! you have all powers !

# Response

I have applied a "brute force" CSS reset for the dashboard grid on mobile screens (< 480px) in `Textbook/src/css/custom.css`.

Changes include:
1.  **Container Reset:** Forced `.dashboard-grid` to be a flex column with 100% width, no margin, and hidden horizontal overflow.
2.  **Card Hard Reset:** Forced all cards (`.card`, `.dashboard-span-*`) to have:
    *   `width: 100% !important`
    *   `max-width: 100% !important`
    *   `box-sizing: border-box !important` (Critical for padding)
    *   `margin: 0 !important`
    *   `overflow: hidden !important` (Crops any internal absolute elements)
    *   `padding: 1.25rem !important` (Overrides inline 2rem padding)
3.  **Typography Adjustments:** Further reduced font sizes and added `word-break: break-word` to headers to prevent long text from pushing the container width out.

This should definitively solve the "cutting" issue by forcing the cards to respect the viewport width and hiding anything that tries to escape.
