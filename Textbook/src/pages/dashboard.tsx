import React from 'react';
import AppLayout from '../components/AppLayout/AppLayout';
import { useAuth } from '../components/Auth/AuthProvider';
import { useHistory } from '@docusaurus/router';
import { 
  Flame, 
  BookOpen, 
  MessageSquare, 
  Terminal, 
  Award, 
  TrendingUp
} from 'lucide-react';

function DashboardContent() {
    const { profile } = useAuth();
    const history = useHistory();

    return (
        <div>
            <header style={{marginBottom: '2rem'}}>
                <h1 style={{fontSize: '2rem', marginBottom: '0.5rem'}}>
                    Dashboard
                </h1>
                <p className="text--secondary">Overview of your learning activity</p>
            </header>

            {/* Bento Grid */}
            <div className="dashboard-grid">
                
                {/* Hero Tile (Welcome) - Span 8 */}
                <div className="card shadow--md dashboard-span-8" style={{
                    background: 'linear-gradient(135deg, rgba(99, 102, 241, 0.1) 0%, rgba(139, 92, 246, 0.05) 100%)',
                    border: '1px solid rgba(99, 102, 241, 0.2)',
                    display: 'flex',
                    flexDirection: 'column',
                    justifyContent: 'center',
                    position: 'relative',
                    overflow: 'hidden',
                    minHeight: '220px'
                }}>
                    <div style={{position: 'relative', zIndex: 2}}>
                        <div style={{display: 'flex', alignItems: 'center', gap: '12px', marginBottom: '1rem'}}>
                            <div style={{padding: '8px 12px', background: 'rgba(245, 158, 11, 0.15)', borderRadius: '20px', border: '1px solid rgba(245, 158, 11, 0.3)', color: '#fbbf24', display: 'flex', alignItems: 'center', gap: '6px', fontSize: '0.85rem', fontWeight: 600}}>
                                <Flame size={16} fill="currentColor" /> 3 Day Streak
                            </div>
                        </div>
                        <h2 style={{fontSize: '2.2rem', marginBottom: '0.5rem'}}>
                            Welcome back, {profile?.name?.split(' ')[0] || 'Explorer'}
                        </h2>
                        <p style={{fontSize: '1.1rem', color: '#cbd5e1', maxWidth: '80%'}}>
                            You're on track to complete <strong>Chapter 2: Embodied AI</strong> by Friday.
                        </p>
                    </div>
                    {/* Abstract BG Pattern */}
                    <div style={{
                        position: 'absolute', right: '-50px', bottom: '-50px', width: '300px', height: '300px',
                        background: 'radial-gradient(circle, rgba(99, 102, 241, 0.15) 0%, transparent 70%)',
                        filter: 'blur(40px)', zIndex: 1
                    }} />
                </div>

                {/* Progress Tile - Span 4 */}
                <div className="card shadow--md dashboard-span-4" style={{display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', textAlign: 'center', minHeight: '200px'}}>
                    <div style={{position: 'relative', width: '120px', height: '120px', marginBottom: '1rem'}}>
                        <svg viewBox="0 0 36 36" style={{width: '100%', height: '100%', transform: 'rotate(-90deg)'}}>
                            <path d="M18 2.0845 a 15.9155 15.9155 0 0 1 0 31.831 a 15.9155 15.9155 0 0 1 0 -31.831" fill="none" stroke="#1e1e2a" strokeWidth="3" />
                            <path d="M18 2.0845 a 15.9155 15.9155 0 0 1 0 31.831 a 15.9155 15.9155 0 0 1 0 -31.831" fill="none" stroke="#6366f1" strokeWidth="3" strokeDasharray="12, 100" strokeLinecap="round" />
                        </svg>
                        <div style={{position: 'absolute', top: '50%', left: '50%', transform: 'translate(-50%, -50%)', fontSize: '1.5rem', fontWeight: 700}}>12%</div>
                    </div>
                    <h3 style={{fontSize: '1.1rem', marginBottom: '0'}}>Overall Progress</h3>
                    <p className="text--secondary" style={{fontSize: '0.85rem'}}>Level: {profile?.python_proficiency || 'Beginner'}</p>
                </div>

                {/* Quick Actions - Span 4 */}
                <div className="card shadow--md dashboard-span-4" style={{padding: '0', minHeight: '200px'}}>
                    <div className="card__header" style={{paddingBottom: '0.5rem'}}>
                        <h3 style={{margin: 0, fontSize: '1rem', color: '#94a3b8', textTransform: 'uppercase', letterSpacing: '0.05em'}}>Quick Actions</h3>
                    </div>
                    <div style={{display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '1px', background: 'rgba(255,255,255,0.05)'}}>
                        <button onClick={() => history.push('/docs/part1/foundations-physical-ai')} style={{aspectRatio: '1', background: 'var(--glass-bg)', border: 'none', display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', cursor: 'pointer', transition: 'background 0.2s', color: '#f8fafc'}}>
                            <BookOpen size={24} style={{marginBottom: '8px', color: '#818cf8'}} />
                            <span style={{fontSize: '0.9rem', fontWeight: 500}}>Read</span>
                        </button>
                        <button style={{aspectRatio: '1', background: 'var(--glass-bg)', border: 'none', display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', cursor: 'pointer', transition: 'background 0.2s', color: '#f8fafc'}}>
                            <Terminal size={24} style={{marginBottom: '8px', color: '#f472b6'}} />
                            <span style={{fontSize: '0.9rem', fontWeight: 500}}>Lab</span>
                        </button>
                        <button style={{aspectRatio: '1', background: 'var(--glass-bg)', border: 'none', display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', cursor: 'pointer', transition: 'background 0.2s', color: '#f8fafc'}}>
                            <Award size={24} style={{marginBottom: '8px', color: '#fbbf24'}} />
                            <span style={{fontSize: '0.9rem', fontWeight: 500}}>Certs</span>
                        </button>
                    </div>
                </div>

                {/* Activity Graph - Span 8 */}
                <div className="card shadow--md dashboard-span-8" style={{display: 'flex', flexDirection: 'column', minHeight: '240px'}}>
                    <div className="card__header" style={{display: 'flex', justifyContent: 'space-between', alignItems: 'center'}}>
                        <h3 style={{margin: 0, fontSize: '1rem', color: '#94a3b8', textTransform: 'uppercase', letterSpacing: '0.05em'}}>Learning Activity</h3>
                        <div style={{display: 'flex', alignItems: 'center', gap: '6px', fontSize: '0.85rem', color: '#10b981'}}>
                            <TrendingUp size={14} /> +24% this week
                        </div>
                    </div>
                    <div className="card__body" style={{flex: 1, display: 'flex', alignItems: 'flex-end', justifyContent: 'space-between', paddingBottom: '0', overflowX: 'auto', gap: '8px'}}>
                        {/* Mock Graph Bars */}
                        {[40, 25, 60, 30, 80, 45, 20, 90, 50, 70, 60, 85].map((h, i) => (
                            <div key={i} style={{
                                width: '24px', /* Fixed width for consistent look */
                                minWidth: '24px',
                                height: `${h}%`, 
                                background: h > 50 ? '#6366f1' : 'rgba(99, 102, 241, 0.3)', 
                                borderRadius: '4px 4px 0 0',
                                transition: 'height 0.5s ease'
                            }} />
                        ))}
                    </div>
                </div>

            </div>
        </div>
    );
}

export default function Dashboard() {
    return (
        <AppLayout title="Dashboard">
            <DashboardContent />
        </AppLayout>
    );
}