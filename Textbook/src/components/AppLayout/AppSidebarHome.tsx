import React from 'react';
import { useHistory } from '@docusaurus/router';
import { Home } from 'lucide-react';

export default function AppSidebarHome() {
  const history = useHistory();

  return (
    <div 
      className="app-sidebar__home-btn" 
      onClick={() => history.push('/')}
      title="Back to Homepage"
    >
      <Home size={20} />
      <span>Home</span>
    </div>
  );
}