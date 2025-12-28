import React from 'react';
import { Redirect } from '@docusaurus/router';
import { useAuth } from './AuthProvider';

export const ProtectedRoute = ({ children }: { children: React.ReactNode }) => {
    const { session, profile, loading } = useAuth();

    if (loading) {
        return <div className="container margin-vert--lg">Loading...</div>;
    }

    if (!session) {
        return <Redirect to="/signin" />;
    }

    if (!profile?.is_onboarded) {
        return <Redirect to="/onboarding" />;
    }

    return <>{children}</>;
};
