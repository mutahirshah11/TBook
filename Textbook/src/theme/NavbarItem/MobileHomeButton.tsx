import React from 'react';
import Link from '@docusaurus/Link';
import { useLocation } from '@docusaurus/router';
import { ChevronRight } from 'lucide-react';
import clsx from 'clsx';
// We import the CSS to get the styled classes. 
// Ensure this doesn't conflict with global styles.
import '../../components/AppLayout/AppLayout.css';

export default function MobileHomeButton(props: any) {
  const location = useLocation();

  // Hide on homepage
  if (location.pathname === '/') {
      return null;
  }

  return (
    <Link
      to="/"
      className={clsx('app-sidebar__item', 'mobile-home-button', 'force-sidebar-only', props.className)}
      style={{
          margin: '0.5rem 0',
          border: '1px solid rgba(255, 255, 255, 0.08)',
          backgroundColor: 'rgba(255, 255, 255, 0.03)'
      }}
    >
      <div className="app-sidebar__icon-wrapper">
        <ChevronRight size={20} />
      </div>
      <span className="app-sidebar__label">Back to Home</span>
    </Link>
  );
}
