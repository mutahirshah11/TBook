import React, { createContext, useContext, useEffect, useState } from 'react';
import { authClient } from '../../lib/auth-client';
import { useHistory, useLocation } from '@docusaurus/router';

interface AuthContextType {
    session: any;
    profile: any;
    loading: boolean;
    refreshProfile: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType>({ 
    session: null, 
    profile: null, 
    loading: true,
    refreshProfile: async () => {} 
});

export const useAuth = () => useContext(AuthContext);

export const AuthProvider = ({ children }: { children: React.ReactNode }) => {
    const [session, setSession] = useState<any>(null);
    const [sessionLoading, setSessionLoading] = useState(true);
    
    const [profile, setProfile] = useState(null);
    const [profileLoading, setProfileLoading] = useState(false);
    const history = useHistory();
    const location = useLocation();

    // Init Session
    useEffect(() => {
        let mounted = true;
        const fetchSession = async () => {
            try {
                const res = await authClient.getSession();
                // Handle potentially different return shapes ({data: session} vs session)
                const sessionData = (res as any)?.data !== undefined ? (res as any).data : res;
                
                if (mounted) {
                    setSession(sessionData);
                }
            } catch (e) {
                console.error("Failed to get session", e);
            } finally {
                if (mounted) {
                    setSessionLoading(false);
                }
            }
        };
        fetchSession();
        return () => { mounted = false; };
    }, []);

    const fetchProfile = async () => {
        if (session?.user) {
            try {
                // Fetch profile from local backend
                const controller = new AbortController();
                const timeoutId = setTimeout(() => controller.abort(), 5000); 

                const res = await fetch('http://localhost:8001/api/profile/me', {
                    credentials: 'include',
                    signal: controller.signal
                });
                
                clearTimeout(timeoutId);

                if (res.ok) {
                    const data = await res.json();
                    setProfile(data);
                } else {
                    console.error("Failed to fetch profile:", res.status);
                    // Fallback to session data if backend fails
                    setProfile({ 
                        ...session.user,
                        is_onboarded: true 
                    } as any);
                }
            } catch (e) {
                console.error("Failed to fetch profile", e);
                // Fallback to session data on error, ensuring session exists
                if (session?.user) {
                    setProfile({ 
                        ...session.user,
                        is_onboarded: true 
                    } as any);
                } else {
                    setProfile(null);
                }
            } finally {
                setProfileLoading(false);
            }
        } else {
            setProfile(null);
            setProfileLoading(false);
        }
    };

    useEffect(() => {
        if (!sessionLoading) {
             // Initial profile fetch
             if (session?.user && !profile) {
                 setProfileLoading(true);
                 fetchProfile();
             } else if (!session?.user) {
                 setProfileLoading(false);
             }
        }
    }, [session, sessionLoading]);

    // Global Onboarding Redirect
    useEffect(() => {
        // Only redirect if we have loaded everything and have a session
        if (!sessionLoading && !profileLoading && session && profile) {
            // If logged in, but not onboarded, and not currently on onboarding page
            if (!profile.is_onboarded && location.pathname !== '/onboarding') {
                history.push('/onboarding');
            }
        }
    }, [session, profile, sessionLoading, profileLoading, location.pathname]);

    return (
        <AuthContext.Provider value={{ 
            session, 
            profile, 
            loading: sessionLoading || profileLoading,
            refreshProfile: fetchProfile
        }}>
            {children}
        </AuthContext.Provider>
    );
};