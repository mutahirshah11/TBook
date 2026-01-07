import { Analytics } from "@vercel/analytics/react"
import React, { useEffect } from 'react';
import ChatbotProvider from '../components/ChatbotUI/ChatbotProvider';
import { AuthProvider, useAuth } from '../components/Auth/AuthProvider';
import { ToastContainer, toast } from 'react-toastify';
import 'react-toastify/dist/ReactToastify.css';
import { useLocation, useHistory } from '@docusaurus/router';
import LoadingSpinner from '../components/UI/LoadingSpinner';

function AuthGuard({ children }: { children: React.ReactNode }) {
    const { session, loading } = useAuth();
    const location = useLocation();
    const history = useHistory();
    // Check if current path is protected
    const isProtected = location.pathname.startsWith('/docs');

    useEffect(() => {
        if (!loading && !session && isProtected) {
             toast.info("Please Login to access the Textbook", {
                toastId: 'auth-guard',
                position: "top-center"
            });
            history.push('/signin');
        }
    }, [session, loading, isProtected, history]);

    // PREVENT FLASH: If loading or (not authenticated AND on protected route), do not render children
    if (loading) {
        return <LoadingSpinner />;
    }

    if (!session && isProtected) {
        return <LoadingSpinner />; // Show spinner while redirecting too
    }

    return <>{children}</>;
}

// Default theme Root component wrapper
export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <AuthProvider>
        <ChatbotProvider>
            <AuthGuard>
                {children}
            </AuthGuard>
            <ToastContainer position="bottom-right" theme="colored" />
            <Analytics /> 
        </ChatbotProvider>
    </AuthProvider>
  );
}
