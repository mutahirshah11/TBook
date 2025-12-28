import React, { useState } from 'react';
import { authClient } from '../lib/auth-client';
import { useHistory } from '@docusaurus/router';
import Link from '@docusaurus/Link';
import { toast } from 'react-toastify';
import { Mail, Lock, User, ArrowRight, Loader2 } from 'lucide-react';
import AuthLayout from '../components/Auth/AuthLayout';

export default function Signup() {
    const [email, setEmail] = useState('');
    const [name, setName] = useState('');
    const [password, setPassword] = useState('');
    const [loading, setLoading] = useState(false);
    const history = useHistory();

    const handleSignup = async (e: React.FormEvent) => {
        e.preventDefault();
        setLoading(true);
        const { data, error } = await authClient.signUp.email({
            email,
            password,
            name,
            callbackURL: "/dashboard"
        });
        
        if (error) {
            setLoading(false);
            toast.error(error.message);
        } else {
            toast.success("Account created successfully!");
            history.push('/dashboard');
        }
    };

    return (
        <AuthLayout 
            title="Create Account" 
            subtitle="Join the future of Physical AI learning"
        >
            <form onSubmit={handleSignup}>
                <div className="margin-bottom--md">
                    <label style={{marginBottom: '0.5rem', display: 'block', fontSize: '0.9rem'}}>Full Name</label>
                    <div style={{position: 'relative'}}>
                        <div style={{position: 'absolute', left: '12px', top: '50%', transform: 'translateY(-50%)', color: '#64748b'}}>
                            <User size={18} />
                        </div>
                        <input 
                            type="text" 
                            value={name} 
                            onChange={e => setName(e.target.value)} 
                            required 
                            placeholder="John Doe"
                            style={{paddingLeft: '40px', width: '100%'}}
                        />
                    </div>
                </div>
                <div className="margin-bottom--md">
                    <label style={{marginBottom: '0.5rem', display: 'block', fontSize: '0.9rem'}}>Email Address</label>
                    <div style={{position: 'relative'}}>
                        <div style={{position: 'absolute', left: '12px', top: '50%', transform: 'translateY(-50%)', color: '#64748b'}}>
                            <Mail size={18} />
                        </div>
                        <input 
                            type="email" 
                            value={email} 
                            onChange={e => setEmail(e.target.value)} 
                            required 
                            placeholder="name@company.com"
                            style={{paddingLeft: '40px', width: '100%'}}
                        />
                    </div>
                </div>
                <div className="margin-bottom--lg">
                    <label style={{marginBottom: '0.5rem', display: 'block', fontSize: '0.9rem'}}>Password</label>
                    <div style={{position: 'relative'}}>
                        <div style={{position: 'absolute', left: '12px', top: '50%', transform: 'translateY(-50%)', color: '#64748b'}}>
                            <Lock size={18} />
                        </div>
                        <input 
                            type="password" 
                            value={password} 
                            onChange={e => setPassword(e.target.value)} 
                            required 
                            placeholder="••••••••"
                            style={{paddingLeft: '40px', width: '100%'}}
                        />
                    </div>
                </div>
                
                <button className="button button--primary button--block" disabled={loading} style={{display: 'flex', alignItems: 'center', justifyContent: 'center', gap: '8px', padding: '12px'}}>
                    {loading ? <Loader2 className="animate-spin" size={20} /> : 'Create Account'}
                    {!loading && <ArrowRight size={18} />}
                </button>
                <style>{`
                    .animate-spin {
                        animation: spin 1s linear infinite;
                    }
                    @keyframes spin {
                        from { transform: rotate(0deg); }
                        to { transform: rotate(360deg); }
                    }
                `}</style>
            </form>

            <div className="text--center margin-top--lg padding-top--md" style={{borderTop: '1px solid rgba(255,255,255,0.05)'}}>
                <p className="margin-bottom--none" style={{fontSize: '0.9rem', color: '#94a3b8'}}>
                    Already have an account? <Link to="/signin" style={{fontWeight: 600}}>Sign In</Link>
                </p>
            </div>
        </AuthLayout>
    );
}