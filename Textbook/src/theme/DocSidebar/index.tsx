import React from 'react';
import clsx from 'clsx';
import {
  useThemeConfig,
} from '@docusaurus/theme-common';
import DefaultDocSidebar from '@theme-original/DocSidebar';

import styles from './styles.module.css';

export default function DocSidebar(props) {
  const themeConfig = useThemeConfig();

  return (
    <DefaultDocSidebar
      {...props}
      className={clsx(props.className, styles.futuristicSidebar)}
    />
  );
}