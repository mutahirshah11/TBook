import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import {
  PageMetadata,
  SkipToContentFallbackId,
  useScrollPosition,
} from '@docusaurus/theme-common';
import {
  useDocsSidebar,
  useDocRouteMetadata,
} from '@docusaurus/theme-common/internal';
import ErrorBoundary from '@docusaurus/ErrorBoundary';
import Navbar from '@theme/Navbar';
import Footer from '@theme/Footer';
import LayoutProvider from '@theme/Layout/Provider';
import ErrorPageContent from '@theme/ErrorPageContent';
import styles from './styles.module.css';
import FloatingControls from '../../../components/ReaderControls/FloatingControls';

import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import {
  PageMetadata,
  SkipToContentFallbackId,
  useScrollPosition,
} from '@docusaurus/theme-common';
import {
  useDocsSidebar,
  useDocRouteMetadata,
} from '@docusaurus/theme-common/internal';
import ErrorBoundary from '@docusaurus/ErrorBoundary';
import Navbar from '@theme/Navbar';
import Footer from '@theme/Footer';
import DocSidebar from '@theme/DocSidebar';
import BackToTopButton from '@theme/BackToTopButton';
import LayoutProvider from '@theme/Layout/Provider';
import ErrorPageContent from '@theme/ErrorPageContent';
import styles from './styles.module.css';
import FloatingControls from '../../../components/ReaderControls/FloatingControls';

function DocPageLayoutContent({
  children,
  className,
}: {
  children: React.ReactNode;
  className?: string;
}) {
  const {sidebarName, pageSidebar} = useDocRouteMetadata();
  const {
    isHidden: sidebarHidden,
    mobileSidebarShown: mobileSidebarShown,
    toggleSidebar,
  } = useDocsSidebar();
  
  const [isZen, setIsZen] = useState(false);

  useEffect(() => {
    if (isZen) {
      document.body.classList.add('zen-mode');
    } else {
      document.body.classList.remove('zen-mode');
    }
    return () => document.body.classList.remove('zen-mode');
  }, [isZen]);

  useScrollPosition(({scrollY}) => {
    if (scrollY > 0) {
      document.body.classList.add(styles.scrolled);
    } else {
      document.body.classList.remove(styles.scrolled);
    }
  });

  return (
    <LayoutProvider>
      <PageMetadata>
        <html className={clsx(mobileSidebarShown && styles.sidebarHidden)} />
      </PageMetadata>

      <Navbar />

      <div
        className={clsx(
          styles.docPage,
          className,
          sidebarHidden && styles.docPageSidebarHidden,
          mobileSidebarShown && styles.docPageSidebarMobileOpen,
          isZen && 'doc-page--zen'
        )}>
        
        {pageSidebar && !isZen && (
          <DocSidebar
            key={sidebarName} // Reset sidebar state when switching sides
            sidebar={pageSidebar}
            path={window.location.pathname}
            onCollapse={toggleSidebar}
            isHidden={sidebarHidden}
          />
        )}

        <main
          id={SkipToContentFallbackId}
          className={clsx(
            styles.docMainContainer,
            (!pageSidebar || isZen) && styles.docMainContainerEnhanced,
          )}>
          <div className={styles.docItemWrapper}>
            {children}
            <BackToTopButton />
          </div>
        </main>
      </div>

      {!isZen && <Footer />}
      
      <FloatingControls 
        isZen={isZen} 
        onZenToggle={() => setIsZen(!isZen)} 
      />
    </LayoutProvider>
  );
}

export default function DocPageLayout(props) {
  return (
    <ErrorBoundary
      fallback={(params) => <ErrorPageContent {...params} />}>
      <DocPageLayoutContent {...props} />
    </ErrorBoundary>
  );
}