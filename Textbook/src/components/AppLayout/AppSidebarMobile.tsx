import React, { useState, useEffect } from 'react';
import { Menu, X, Cpu } from 'lucide-react';
import AppSidebar from './AppSidebar';
import './AppLayout.css';

export default function AppSidebarMobile() {
  const [isOpen, setIsOpen] = useState(false);

  // Close sidebar when clicking outside or navigating
  useEffect(() => {
    const handleClickOutside = (e: MouseEvent) => {
      const sidebar = document.querySelector('.app-sidebar-mobile-drawer');
      const toggle = document.querySelector('.app-mobile-toggle');
      if (isOpen && sidebar && !sidebar.contains(e.target as Node) && !toggle?.contains(e.target as Node)) {
        setIsOpen(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, [isOpen]);

  return (
    <>
      <div className="app-mobile-header">
        <button 
          className="app-mobile-toggle" 
          onClick={() => setIsOpen(!isOpen)}
        >
          {isOpen ? <X size={24} /> : <Menu size={24} />}
        </button>
        <div className="app-mobile-brand">
            <img src="/img/logo.svg" alt="RoboLearn" style={{height: '28px', width: 'auto', marginRight: '8px'}} />
            <span>RoboLearn</span>
        </div>
      </div>

      <div className={`app-sidebar-mobile-drawer ${isOpen ? 'open' : ''}`}>
        <AppSidebar />
      </div>
      
      {isOpen && <div className="app-mobile-backdrop" onClick={() => setIsOpen(false)} />}
    </>
  );
}