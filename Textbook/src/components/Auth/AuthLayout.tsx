import React from 'react';
import Layout from '@theme/Layout';
import './AuthLayout.css'; // We'll create this

interface AuthLayoutProps {
  children: React.ReactNode;
  title: string;
  subtitle: string;
}

export default function AuthLayout({ children, title, subtitle }: AuthLayoutProps) {
  return (
    <Layout title={title} noFooter>
      <div className="auth-page">
        {/* Left Side: High-Fidelity Visual */}
        <div className="auth-page__visual">
            <div className="auth-page__visual-content">
                <div className="auth-page__logo-mark">
                    <div className="auth-page__logo-orb" />
                </div>
                <h2 className="auth-page__visual-title">The Future of <br />Humanoid Robotics</h2>
                <p className="auth-page__visual-text">
                    Join a community of 5,000+ engineers mastering the intersection of Physical AI and Embodied Intelligence.
                </p>
                <div className="auth-page__visual-footer">
                    <div className="auth-page__trust-badge">
                        <span>POWERED BY</span>
                        <img src="/img/logo.svg" alt="Logo" style={{height: '20px', marginLeft: '8px'}} />
                    </div>
                </div>
            </div>
            
            {/* Background elements */}
            <div className="auth-page__visual-glow" />
            <div className="auth-page__visual-grid" />
        </div>

        {/* Right Side: Form */}
        <div className="auth-page__form-side">
            <div className="auth-page__form-container">
                <div className="auth-page__form-header">
                    <h1 className="auth-page__form-title">{title}</h1>
                    <p className="auth-page__form-subtitle">{subtitle}</p>
                </div>
                
                {children}
            </div>
        </div>
      </div>
    </Layout>
  );
}
