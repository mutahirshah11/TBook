import React, { useState, useRef, useEffect } from 'react';
import NavbarItem from '@theme-original/NavbarItem';
import { useAuth } from '@site/src/components/Auth/AuthProvider';
import { useHistory } from '@docusaurus/router';
import { authClient } from '@site/src/lib/auth-client';
import { toast } from 'react-toastify';
import MobileHomeButton from './MobileHomeButton';

function UserMenu({ session, className }) {
    const [isOpen, setIsOpen] = useState(false);
    const dropdownRef = useRef(null);
    const user = session?.user;
    const initial = user?.name ? user.name.charAt(0).toUpperCase() : (user?.email?.charAt(0).toUpperCase() || 'U');

    useEffect(() => {
        function handleClickOutside(event) {
            if (dropdownRef.current && !dropdownRef.current.contains(event.target)) {
                setIsOpen(false);
            }
        }
        document.addEventListener("mousedown", handleClickOutside);
        return () => {
            document.removeEventListener("mousedown", handleClickOutside);
        };
    }, [dropdownRef]);

    const handleLogout = async () => {
         const toastId = toast.loading("Logging out...", { 
             position: "top-center",
             className: 'custom-logout-toast'
         });
         await authClient.signOut();
         toast.dismiss(toastId);
         window.location.href = '/';
    };

    return (
        <div className={`navbar__item user-profile-container ${className || ''}`} ref={dropdownRef} style={{position: 'relative'}}>
            <div className="user-profile-circle" onClick={() => setIsOpen(!isOpen)} title={user?.name || user?.email}>
                {initial}
            </div>
            {isOpen && (
                <div className="user-profile-dropdown">
                    <div className="user-profile-item" style={{cursor: 'default', opacity: 0.7, fontSize: '0.8rem', borderBottom: '1px solid rgba(255,255,255,0.1)', marginBottom: '4px', paddingBottom: '8px'}}>
                         {user?.name || user?.email}
                    </div>
                    <div className="user-profile-item logout" onClick={handleLogout}>
                        Logout
                    </div>
                </div>
            )}
        </div>
    );
}

export default function NavbarItemWrapper(props) {
  const { session, loading } = useAuth();
  const history = useHistory();
  const { label } = props;

  // Don't render anything until we know auth state (to avoid flicker)
  // Although for SSG this might mean navbar is empty initially. 
  // Better to render "Guest" view by default if loading?
  // Or just let it update.
  
  if (label === 'Dashboard') {
    if (!session) return null;
  }

  // Handle Sign In: Show UserMenu if logged in, else show Sign In link
  if (label === 'Login') {
    if (session) {
        // Force hide in mobile sidebar - Docusaurus v3 uses 'mobile' prop for items in sidebar
        if (props.mobile === true) {
            return null;
        }
        return <UserMenu session={session} className={props.className} />;
    }
    return <NavbarItem {...props} />;
  }

  // Handle Get Started / Sign Up: Hide if logged in
  if (label === 'Get Started' || label === 'Sign Up') {
    if (session) return null;

    // Extra filtering to prevent duplicates in mobile sidebar
    if (props.mobile === true) {
        // Hide the Desktop-only Sign Up button in mobile sidebar
        if (props.className?.includes('desktop-only-item')) {
            return null;
        }
        // Hide the Mobile-Navbar-only Get Started button in mobile sidebar
        if (props.className?.includes('mobile-navbar-cta')) {
            return null;
        }
    }

    return <NavbarItem {...props} />;
  }

  if (label === 'Profile') {
      if (!session) return null;
      if (props.mobile === true) return null;
      return <NavbarItem {...props} to="/dashboard" />;
  }

  if (label === 'Back to Home') {
      // Rely on CSS (.mobile-only-nav-item) to hide on desktop
      // We render it always, allowing CSS to control visibility
      return <MobileHomeButton {...props} />;
  }

  if (label === 'Logout') {
    if (!session) return null;
    
    // Hide Logout in mobile sidebar
    if (props.mobile === true) {
        return null;
    }
    
    return (
        <div 
            className={`navbar__item navbar__link ${props.className || ''}`}
            style={{cursor: 'pointer'}} 
            onClick={async () => {
                const toastId = toast.loading("Logging out...", { 
                    position: "top-center",
                    className: 'custom-logout-toast'
                });
                await authClient.signOut();
                toast.dismiss(toastId);
                window.location.href = '/';
            }}
        >
            Logout
        </div>
    );
  }

  if (label === 'Textbook') {
      // If not logged in, intercept click
      if (!session) {
          return (
            <a 
                href="#" 
                className="navbar__item navbar__link"
                onClick={(e) => {
                    e.preventDefault();
                    toast.info("Please Login to access the Textbook", {
                        position: "top-center",
                        autoClose: 3000,
                    });
                }}
            >
                Textbook
            </a>
          );
      }
  }

  return <NavbarItem {...props} />;
}