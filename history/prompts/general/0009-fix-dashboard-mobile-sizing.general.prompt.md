---
id: "0009"
title: Fix Dashboard Mobile Card Sizing
stage: refactor
date: 2025-12-28
surface: agent
model: gemini-2.0-flash-exp
feature: general
branch: main
user: Mutahir
command: fix mobile card cutting
labels: [css, responsive, dashboard, mobile]
links: null
files:
 - Textbook/src/css/custom.css
tests: []
---

# Prompt

cards are still cutting you can make the cards a little smaller for mobile screen sizez so dont cut

# Response

I have updated `Textbook/src/css/custom.css` to aggressively fix the card sizing on mobile screens (< 480px):

1.  **Reduced Padding:** Added `padding: 1rem !important` to override the inline `2rem` padding on cards. This recovers 32px of horizontal space on each card.
2.  **Constrained Width:** Re-affirmed `width: 100% !important` and `max-width: 100% !important`.
3.  **Flexible Height:** Set `min-height: auto !important` to allow cards to compact vertically if possible.
4.  **Font Sizing:** Reduced font sizes for `h2`, `h3`, and `p` elements inside dashboard cards to ensure text fits better and doesn't push boundaries.

These changes should ensure the cards fit comfortably within the viewport without being cut off.
