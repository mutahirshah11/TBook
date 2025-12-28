import React from 'react';
import Layout from '@theme/Layout';
import AppSidebar from './AppSidebar';
import AppSidebarMobile from './AppSidebarMobile';
import './AppLayout.css';
import { ProtectedRoute } from '../Auth/ProtectedRoute';

interface AppLayoutProps {
  children: React.ReactNode;
  title?: string;
}

export default function AppLayout({ children, title }: AppLayoutProps) {
  return (
    <Layout 
      title={title} 
      wrapperClassName="app-shell-wrapper" // Hooks into custom CSS to hide standard nav
      noFooter // Built-in prop to hide footer
    >
      <ProtectedRoute>
        <div className="app-shell">
            <AppSidebar />
            <AppSidebarMobile />
            <main className="app-content">
                <div className="app-content__inner">
                    {children}
                </div>
            </main>
        </div>
      </ProtectedRoute>
    </Layout>
  );
}
