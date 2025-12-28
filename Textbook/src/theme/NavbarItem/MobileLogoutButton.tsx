import React from 'react';
import { useAuth } from '@site/src/components/Auth/AuthProvider';
import { authClient } from '@site/src/lib/auth-client';
import clsx from 'clsx';

export default function MobileLogoutButton(props: any) {
  const { session } = useAuth();
  
  // Only render if logged in
  if (!session) return null;

  return (
    <li className="menu__list-item mobile-only-item">
      <button
        className="menu__link"
        style={{
             background: 'none', 
             border: 'none', 
             cursor: 'pointer', 
             width: '100%', 
             textAlign: 'left',
             color: '#ef4444', // Red color for logout
             display: 'flex',
             alignItems: 'center'
        }}
        onClick={async () => {
          await authClient.signOut();
          window.location.href = '/';
        }}
      >
        Sign Out
      </button>
    </li>
  );
}
