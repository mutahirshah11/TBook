import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';
import { toast } from 'react-toastify';
import { useAuth } from '../components/Auth/AuthProvider';
import { config } from '../config';

export default function Onboarding() {
  const [proficiency, setProficiency] = useState('Beginner');
  const [role, setRole] = useState('Frontend Developer');
  const [loading, setLoading] = useState(false);
  const [success, setSuccess] = useState(false);
  const history = useHistory();
  const { refreshProfile, profile, session } = useAuth();

  // Redirect if already onboarded
  React.useEffect(() => {
    if (session && profile?.is_onboarded) {
      history.push('/dashboard');
    }
  }, [profile, session, history]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    try {
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), 8000); 

        const res = await fetch(`${config.backendUrl}/api/profile/onboarding`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ python_proficiency: proficiency, developer_role: role }),
            credentials: 'include', 
            signal: controller.signal
        });
        
        clearTimeout(timeoutId);
        
        if (res.ok) {
            setSuccess(true);
            toast.success("Profile Personalized Successfully!");
            await refreshProfile(); // Refresh context
            setTimeout(() => history.push('/dashboard'), 1500); // Redirect after delay
        } else if (res.status === 403) {
            // Already onboarded - treat as success
            const err = await res.json().catch(() => ({}));
            if (err.detail === "Profile already completed") {
                 setSuccess(true);
                 toast.info("Profile Already Personalized!");
                 await refreshProfile();
                 setTimeout(() => history.push('/dashboard'), 1500);
            } else {
                 toast.error("Error: " + (err.detail || "Forbidden"));
            }
        } else {
            const err = await res.json().catch(() => ({ detail: res.statusText }));
            toast.error("Error: " + (err.detail || "Failed to save profile"));
        }
    } catch (e: any) {
        if (e.name === 'AbortError') {
             toast.error("Request timed out. Backend might be slow or down.");
        } else {
             toast.error("Network error. Could not connect to the profile service.");
        }
    } finally {
        setLoading(false);
    }
  };

  return (
    <Layout title="Onboarding">
       <div className="container margin-vert--lg" style={{ maxWidth: '500px' }}>
         <div className="text--center margin-bottom--lg">
            <h1>Complete Your Profile</h1>
            <p>Tell us about yourself to personalize your AI interactions.</p>
         </div>
         
         <form onSubmit={handleSubmit}>
            <div className="margin-bottom--md">
                <label style={{display:'block', marginBottom:'5px', fontWeight:'bold'}}>Python Proficiency</label>
                <select 
                    className="button button--block button--outline button--secondary" 
                    style={{ cursor: 'pointer', textAlign: 'left' }}
                    value={proficiency} 
                    onChange={e => setProficiency(e.target.value)}
                    disabled={success || loading}
                >
                    <option value="Beginner">Beginner</option>
                    <option value="Intermediate">Intermediate</option>
                    <option value="Advanced">Advanced</option>
                </select>
            </div>
            
            <div className="margin-bottom--lg">
                <label style={{display:'block', marginBottom:'5px', fontWeight:'bold'}}>Developer Role</label>
                <select 
                    className="button button--block button--outline button--secondary" 
                    style={{ cursor: 'pointer', textAlign: 'left' }}
                    value={role} 
                    onChange={e => setRole(e.target.value)}
                    disabled={success || loading}
                >
                    <option value="Frontend Developer">Frontend Developer</option>
                    <option value="Backend Developer">Backend Developer</option>
                    <option value="Full Stack Developer">Full Stack Developer</option>
                    <option value="None">None</option>
                </select>
            </div>
            
            <button 
                className={`button button--block ${success ? 'button--success' : 'button--primary'}`} 
                disabled={loading || success}
                style={{ fontSize: '1.1rem', padding: '12px' }}
            >
                {success ? 'Profile Successfully Personalized!' : (loading ? 'Saving Profile...' : 'Complete Profile')}
            </button>
         </form>
       </div>
    </Layout>
  );
}
