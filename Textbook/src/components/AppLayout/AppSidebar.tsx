import React from 'react';
import Link from '@docusaurus/Link';
import { useLocation, useHistory } from '@docusaurus/router';
import { useAuth } from '../Auth/AuthProvider';
import { authClient } from '../../lib/auth-client';
import { 
  LayoutDashboard, 
  BookOpen, 
  User, 
  LogOut, 
  Settings, 
  MessageSquare, 
  Cpu,
  ChevronRight
} from 'lucide-react';
import clsx from 'clsx';
import './AppLayout.css'; // We'll create this

const SidebarItem = ({ to, icon: Icon, label, active, onClick }: any) => {
  return (
    <Link
      to={to}
      onClick={onClick}
      className={clsx('app-sidebar__item', { 'app-sidebar__item--active': active })}
    >
      <div className="app-sidebar__icon-wrapper">
        <Icon size={20} />
      </div>
      <span className="app-sidebar__label">{label}</span>
      {active && <div className="app-sidebar__active-indicator" />}
    </Link>
  );
};

export default function AppSidebar() {
  const location = useLocation();
  const history = useHistory();
  const { profile } = useAuth();

  const handleSignOut = async () => {
    await authClient.signOut();
    history.push('/');
    window.location.reload();
  };

  const isActive = (path: string) => location.pathname === path;

  return (
    <aside className="app-sidebar">
      <div className="app-sidebar__header">
        <div className="app-sidebar__logo">
            <img src="/img/logo.svg" alt="Logo" style={{width: '100%', height: '100%'}} />
        </div>
        <span className="app-sidebar__brand">RoboLearn</span>
      </div>

      <div className="app-sidebar__user-card">
        <div className="app-sidebar__avatar">
            <img src={profile?.image || `https://ui-avatars.com/api/?name=${profile?.name || 'User'}&background=6366f1&color=fff`} alt="Profile" />
        </div>
        <div className="app-sidebar__user-info">
            <span className="app-sidebar__user-name">{profile?.name || 'Explorer'}</span>
            <span className="app-sidebar__user-role">{profile?.developer_role || 'Student'}</span>
        </div>
      </div>

      <nav className="app-sidebar__nav">
        <SidebarItem 
          to="/" 
          icon={ChevronRight} // Using a simple arrow or Home icon if available, but chevron implies "Back" or "Go"
          label="Back to Home" 
          active={false}
          onClick={() => history.push('/')}
        />
        <div className="app-sidebar__divider" style={{margin: '0.5rem 0 1rem'}} />

        <div className="app-sidebar__section-label">WORKSPACE</div>
        <SidebarItem 
          to="/dashboard" 
          icon={LayoutDashboard} 
          label="Dashboard" 
          active={isActive('/dashboard')} 
        />
        <SidebarItem 
          to="/profile" 
          icon={User} 
          label="Profile" 
          active={isActive('/profile')} 
        />
        
        <div className="app-sidebar__section-label margin-top--md">LEARNING</div>
        <SidebarItem 
          to="/docs/part1/foundations-physical-ai" 
          icon={BookOpen} 
          label="Course Material" 
          active={location.pathname.startsWith('/docs')} 
        />
        <SidebarItem 
          to="/chatbot-test" 
          icon={MessageSquare} 
          label="AI Tutor" 
          active={isActive('/chatbot-test')} 
        />

        <div className="app-sidebar__spacer" />
        
        <div className="app-sidebar__divider" />
        
        <SidebarItem 
          to="#" 
          icon={Settings} 
          label="Settings" 
          active={false}
        />
        <button className="app-sidebar__logout-btn" onClick={handleSignOut}>
            <LogOut size={18} />
            <span>Sign Out</span>
        </button>
      </nav>
    </aside>
  );
}
