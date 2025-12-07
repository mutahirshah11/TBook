import React from 'react';
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
        )}>
        <main
          id={SkipToContentFallbackId}
          className={clsx(
            styles.docMainContainer,
            !pageSidebar && styles.docMainContainerEnhanced,
          )}>
          <div className={styles.docItemWrapper}>{children}</div>
        </main>
      </div>

      <Footer />
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