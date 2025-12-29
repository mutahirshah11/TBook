import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import { Cpu, Zap, Activity, Globe, Layers, Eye, BookOpen, ArrowRight, Star } from 'lucide-react';

import styles from './index.module.css';

function HomepageHeader() {
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {  
    setIsVisible(true);
  }, []);

  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className={styles.heroVisual}>
        <div className={styles.heroOrb}></div>
        <div className={styles.heroGrid}></div>
        <div className={styles.heroFlow}></div>
      </div>
      
      <div className="container">
        <div className={clsx(styles.heroContent, isVisible && styles.visible)}>
          <div className={styles.heroBadge}>
              <Star size={12} style={{marginRight: '6px'}} fill="currentColor" />
              Interactive Curriculum • Powered by RAG AI
          </div>
          <Heading as="h1" className={styles.mainTitle}>
            Physical AI & <br className="desktop-only" />
            <span className={styles.nowrapDesktop}>RAG-Enhanced Learning</span>
          </Heading>
          <p className={styles.subtitle}>
            The world's first "living" textbook for humanoid robotics. Master control systems 
            with a context-aware RAG Agent that answers questions and debugs code in real-time.
          </p>
          <div className={styles.buttons}>
            <Link
              className={clsx('button button--primary button--lg', styles.shimmerButton)}
              to="/docs/part1/foundations-physical-ai"
              style={{display:'flex', alignItems:'center', gap:'10px', padding: '1rem 2rem'}}>
              Start Reading & Chatting <ArrowRight size={20} />
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/signup"
              style={{padding: '1rem 2rem'}}>
              Join Community
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

function CourseStats() {
  const stats = [
    { value: '06', label: 'Modular Parts' },
    { value: '23', label: 'Advanced Chapters' },
    { value: '100%', label: 'RAG Coverage' },
  ];

  return (
    <section className={styles.statsSection}>
      <div className="container">
        <div className="row">
          {stats.map((stat, index) => (
            <div key={index} className="col col--4 text--center">
              <div className={styles.statValue}>{stat.value}</div>
              <div className={styles.statLabel}>{stat.label}</div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// Helper for mouse-tracking glow effect
const SpotlightCard = ({ children, className = "", style = {} }: { children: React.ReactNode; className?: string; style?: React.CSSProperties }) => {
  const handleMouseMove = (e: React.MouseEvent<HTMLDivElement>) => {
    const rect = e.currentTarget.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    e.currentTarget.style.setProperty('--mouse-x', `${x}px`);
    e.currentTarget.style.setProperty('--mouse-y', `${y}px`);
  };

  return (
    <div 
      className={className} 
      style={style} 
      onMouseMove={handleMouseMove}
    >
      {children}
    </div>
  );
};

function TechnologiesSection() {
  const technologies = [
    { name: 'Embodied AI', desc: 'Neural architectures for physical systems and real-time inference.', icon: <Globe size={24} />, span: 'span 8', color: '99, 102, 241' },
    { name: 'Computer Vision', desc: 'Spatial awareness and object tracking.', icon: <Layers size={24} />, span: 'span 4', color: '6, 182, 212' },
    { name: 'Control Systems', desc: 'Precision motion and dynamic balance.', icon: <Cpu size={24} />, span: 'span 4', color: '245, 158, 11' },
    { name: 'Locomotion', desc: 'Bipedal and quadrupedal movement algorithms.', icon: <Activity size={24} />, span: 'span 4', color: '16, 185, 129' },
    { name: 'Sensors', desc: 'Multi-modal data fusion.', icon: <Eye size={24} />, span: 'span 4', color: '244, 63, 94' },
  ];

  return (
    <section className={styles.sectionContainer}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            The AI-Native Stack
          </Heading>
          <p className={styles.sectionSubtitle}>
            Every chapter is integrated with our RAG system, allowing you to query technical 
            details and implementation steps as you learn.
          </p>
        </div>
        
        <div className={styles.bentoGrid}>
          {technologies.map((tech, index) => (
            <SpotlightCard key={index} className={styles.bentoItem} style={{gridColumn: tech.span, '--tech-color': tech.color} as React.CSSProperties}>
              <div className={styles.bentoItemIcon}>
                {tech.icon}
              </div>
              <h3 className={styles.bentoItemTitle}>{tech.name}</h3>
              <p className={styles.bentoItemDesc}>{tech.desc}</p>
            </SpotlightCard>
          ))}
        </div>
      </div>
    </section>
  );
}

function UniqueValueProposition() {
  const values = [
    { title: 'Research First', desc: 'Direct implementation of papers from NVIDIA, Tesla, and Boston Dynamics labs.', icon: <BookOpen size={32} />, color: '99, 102, 241' },
    { title: 'Production Ready', desc: 'Not just theory. Learn the deployment pipelines used in industry robotic fleets.', icon: <Zap size={32} />, color: '236, 72, 153' },
    { title: 'Chat with the Book', desc: 'The integrated RAG Chatbot knows every line of this curriculum. Ask it to simplify equations or generate code instantly.', icon: <Cpu size={32} />, color: '16, 185, 129' },
  ];

  return (
    <section className={styles.sectionContainer} style={{background: 'rgba(255,255,255,0.01)'}}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            Why RoboLearn?
          </Heading>
        </div>
        
        <div className={styles.valueGrid}>
          {values.map((value, index) => (
            <SpotlightCard key={index} className={styles.valueCard} style={{'--tech-color': value.color} as React.CSSProperties}>
              <div className={styles.valueCardIcon}>{value.icon}</div>
              <h3 className={styles.bentoItemTitle}>{value.title}</h3>
              <p className={styles.bentoItemDesc}>{value.desc}</p>
            </SpotlightCard>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  
  useEffect(() => {
    const handleScroll = () => {
      if (window.scrollY > 50) {
        document.body.classList.add('home-page--scrolled');
      } else {
        document.body.classList.remove('home-page--scrolled');
      }
    };
    
    document.body.classList.add('home-page');
    window.addEventListener('scroll', handleScroll);
    
    return () => {
      document.body.classList.remove('home-page', 'home-page--scrolled');
      window.removeEventListener('scroll', handleScroll);
    };
  }, []);

  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="Professional engineering course for embodied intelligence.">
      <HomepageHeader />
      <main>
        <CourseStats />
        <TechnologiesSection />
        
        <div className="container">
          <div className={styles.courseOverview}>
            <h2 className={styles.overviewTitle}>Read, Chat, Build</h2>
            <p className={styles.overviewDescription}>
              Our curriculum isn't just a PDF. It's a searchable, interactive knowledge base. 
              Move from theory to implementation with an AI that's read every word of the syllabus.
            </p>
            <Link
              className="button button--primary button--lg"
              to="/docs/part1/foundations-physical-ai">
              Browse the Syllabus
            </Link>
          </div>
        </div>

        <UniqueValueProposition />
      </main>
    </Layout>
  );
}
