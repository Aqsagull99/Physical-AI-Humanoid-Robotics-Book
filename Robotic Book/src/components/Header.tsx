import type {ReactNode} from 'react';
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import {ThemeClassNames} from '@docusaurus/theme-common';
import {useAnnouncementBar, useScrollPosition} from '@docusaurus/theme-common/internal';
import NavbarItem from '@theme/NavbarItem';
import NavbarColorModeToggle from '@theme/Navbar/ColorModeToggle';
import Content from '@theme/Navbar/Content';
import Logo from '@theme/Logo';
import SearchBar from '@theme/SearchBar';

import styles from './Header.module.css';

function HeaderWrapper(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  const {isAnnouncementBarClosed} = useAnnouncementBar();
  const [navbarRef, setNavbarRef] = React.useState<HTMLElement | null>(null);
  const [hidden, setHidden] = React.useState(false);

  useScrollPosition(
    ({scrollY}) => {
      const {frontMatter} = siteConfig;
      if (frontMatter.hide_table_of_contents) {
        setHidden(scrollY > 0);
      }
    },
    [hidden],
  );

  return (
    <header
      ref={setNavbarRef}
      className={clsx(
        'navbar',
        'navbar--dark',
        'navbar--sticky',
        'navbar--primary',
        ThemeClassNames.header.header,
        styles.header,
        !isAnnouncementBarClosed && styles.headerHideable,
        hidden && styles.headerHidden,
      )}>
      <div className="navbar__inner">
        <div className="navbar__items">
          <Logo
            className="navbar__brand"
            imageClassName="navbar__logo"
            titleClassName="navbar__title text--truncate"
          />
        </div>
        <div className="navbar__items navbar__items--right">
          <Content />
        </div>
      </div>
    </header>
  );
}

export default function Header(): ReactNode {
  return <HeaderWrapper />;
}