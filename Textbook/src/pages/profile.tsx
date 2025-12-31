import React, { useState, useEffect } from 'react';
import AppLayout from '../components/AppLayout/AppLayout';
import { useAuth } from '../components/Auth/AuthProvider';
import { toast } from 'react-toastify';
import { User, Shield, Zap, Mail, Fingerprint, Save } from 'lucide-react';
import { config } from '../config';

function ProfileContent() {
// ...
    const handleUpdate = async (e: React.FormEvent) => {
        e.preventDefault();
        setLoading(true);
        try {
            const res = await fetch(`${config.backendUrl}/api/profile/onboarding`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ python_proficiency: proficiency, developer_role: role }),
                credentials: 'include',
            });

            if (res.ok) {
                toast.success("Profile updated successfully!");
                await refreshProfile();
            } else {
                const data = await res.json();
                toast.error("Update failed: " + (data.detail || "Unknown error"));
            }
        } catch (e) {
            toast.error("Network error while updating profile.");
        } finally {
            setLoading(false);
        }
    };

    return (
        <div>
            <header style={{marginBottom: '2rem'}}>
                <h1 style={{fontSize: '2rem', marginBottom: '0.5rem'}}>Account Settings</h1>
                <p className="text--secondary">Manage your identity and learning preferences</p>
            </header>

            <div className="dashboard-grid">
                {/* User Identity Tile */}
                <div className="card shadow--md dashboard-span-4" style={{display: 'flex', flexDirection: 'column', alignItems: 'center', padding: '2rem'}}>
                    <div style={{position: 'relative', marginBottom: '1.5rem'}}>
                        <div style={{
                            width: '100px', height: '100px', 
                            borderRadius: '30px', overflow: 'hidden',
                            border: '2px solid var(--ifm-color-primary)',
                            padding: '4px', background: 'rgba(99, 102, 241, 0.1)'
                        }}>
                            <img
                                src={profile?.image || "https://ui-avatars.com/api/?name=" + (profile?.name || "User") + "&background=6366f1&color=fff"}
                                alt="Avatar"
                                style={{width: '100%', height: '100%', borderRadius: '24px', objectFit: 'cover'}}
                            />
                        </div>
                        <div style={{
                            position: 'absolute', bottom: '-5px', right: '-5px',
                            background: '#10b981', width: '24px', height: '24px',
                            borderRadius: '50%', border: '3px solid #0a0a0f',
                            boxShadow: '0 0 10px rgba(16, 185, 129, 0.4)'
                        }} />
                    </div>
                    
                    <h2 style={{fontSize: '1.25rem', marginBottom: '0.25rem'}}>{profile?.name || 'User'}</h2>
                    <p style={{fontSize: '0.85rem', color: '#94a3b8', marginBottom: '1.5rem'}}>{profile?.email}</p>
                    
                    <div style={{width: '100%', display: 'flex', flexDirection: 'column', gap: '12px'}}>
                        <div style={{display:'flex', alignItems:'center', gap:'10px', fontSize:'0.85rem', color:'#cbd5e1', padding:'8px 12px', background:'rgba(255,255,255,0.03)', borderRadius:'8px'}}>
                            <Fingerprint size={16} color="#818cf8" />
                            <span>ID: {profile?.id?.substring(0, 12)}...</span>
                        </div>
                        <div style={{display:'flex', alignItems:'center', gap:'10px', fontSize:'0.85rem', color:'#cbd5e1', padding:'8px 12px', background:'rgba(255,255,255,0.03)', borderRadius:'8px'}}>
                            <Shield size={16} color="#34d399" />
                            <span>Account Verified</span>
                        </div>
                    </div>
                </div>

                {/* Personalization Tile */}
                <div className="card shadow--md dashboard-span-8" style={{padding: '2rem'}}>
                    <div style={{display: 'flex', alignItems: 'center', gap: '12px', marginBottom: '2rem'}}>
                        <div style={{width: '40px', height: '40px', background: 'rgba(99, 102, 241, 0.1)', borderRadius: '10px', display: 'flex', alignItems: 'center', justifyContent: 'center', color: '#818cf8'}}>
                            <Zap size={20} />
                        </div>
                        <h3 style={{margin: 0}}>Learning Preferences</h3>
                    </div>

                    <form onSubmit={handleUpdate}>
                        <div className="row">
                            <div className="col col--6 margin-bottom--lg">
                                <label style={{fontSize: '0.85rem', textTransform: 'uppercase', letterSpacing: '0.05em', color: '#64748b'}}>Python Level</label>
                                <select 
                                    className="button button--block" 
                                    style={{ textAlign: 'left', marginTop: '8px', background: 'rgba(0,0,0,0.2)', border: '1px solid rgba(255,255,255,0.1)' }}
                                    value={proficiency} 
                                    onChange={e => setProficiency(e.target.value)}
                                    disabled={loading}
                                >
                                    <option value="Beginner">Beginner</option>
                                    <option value="Intermediate">Intermediate</option>
                                    <option value="Advanced">Advanced</option>
                                </select>
                            </div>
                            
                            <div className="col col--6 margin-bottom--lg">
                                <label style={{fontSize: '0.85rem', textTransform: 'uppercase', letterSpacing: '0.05em', color: '#64748b'}}>Primary Role</label>
                                <select 
                                    className="button button--block" 
                                    style={{ textAlign: 'left', marginTop: '8px', background: 'rgba(0,0,0,0.2)', border: '1px solid rgba(255,255,255,0.1)' }}
                                    value={role} 
                                    onChange={e => setRole(e.target.value)}
                                    disabled={loading}
                                >
                                    <option value="Frontend Developer">Frontend Developer</option>
                                    <option value="Backend Developer">Backend Developer</option>
                                    <option value="Full Stack Developer">Full Stack Developer</option>
                                    <option value="None">None</option>
                                </select>
                            </div>
                        </div>

                        <hr style={{margin: '1.5rem 0', opacity: 0.1}} />

                        <div className="text--right">
                            <button className="button button--primary" disabled={loading} style={{display:'inline-flex', alignItems:'center', gap:'8px'}}>
                                <Save size={18} />
                                {loading ? 'Saving...' : 'Save Changes'}
                            </button>
                        </div>
                    </form>
                </div>

                {/* Info Tile */}
                <div className="card shadow--md dashboard-span-12" style={{gridColumn: 'span 12', background: 'rgba(99, 102, 241, 0.03)', border: '1px dashed rgba(99, 102, 241, 0.2)'}}>
                    <div className="card__body" style={{display: 'flex', alignItems: 'center', gap: '1rem'}}>
                        <div style={{color: '#818cf8'}}>
                            <Mail size={24} />
                        </div>
                        <div>
                            <h4 style={{margin: 0}}>Subscription Tier</h4>
                            <p className="margin-bottom--none text--secondary" style={{fontSize: '0.9rem'}}>You are currently on the <strong>Founder's Plan</strong> with lifetime access.</p>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    );
}

export default function Profile() {
    return (
        <AppLayout title="My Profile">
            <ProfileContent />
        </AppLayout>
    );
}
