import { test, expect } from '@playwright/test';

test('should display a placeholder page at the root URL', async ({ page }) => {
  await page.goto('/');
  // Expect a title "to contain" a substring.
  await expect(page).toHaveTitle(/Robotics Book/);

  // Expect a specific heading or element indicating a placeholder/welcome page
  await expect(page.locator('text=Welcome')).toBeVisible();
  await expect(page.locator('text=Hello Docusaurus')).toBeVisible();
});