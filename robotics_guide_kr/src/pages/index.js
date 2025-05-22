import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';
import HomepageFeatures from '@site/src/components/HomepageFeatures'; // We'll create this component next

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/isaac_sim/overview">
            Isaac Sim 시작하기 - 5분 ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`홈 - ${siteConfig.title}`}
      description="Isaac Sim, Isaac Lab, ROS 2 Humble을 마스터하기 위한 최고의 한국어 가이드. 튜토리얼, 심층 분석, 예제 프로젝트를 통해 로보틱스 시뮬레이션 전문가가 되어보세요.">
      <HomepageHeader />
      <main>
        {/* 
          The HomepageFeatures component will render feature blocks.
          We can also add other sections here directly if needed.
        */}
        <HomepageFeatures />

        <section className={styles.section}>
          <div className="container text--center">
            <h2>주요 학습 경로</h2>
            <p>다음 기술들을 심층적으로 배워보세요:</p>
            <div className="row">
              <div className={clsx('col col--4', styles.featureCard)}>
                <h3>NVIDIA Isaac Sim</h3>
                <p>사실적인 로봇 시뮬레이션 및 합성 데이터 생성을 위한 강력한 플랫폼입니다. 기본부터 고급 활용법까지 알아보세요.</p>
                <Link className="button button--outline button--primary" to="/docs/isaac_sim/overview">
                  Isaac Sim 가이드
                </Link>
              </div>
              <div className={clsx('col col--4', styles.featureCard)}>
                <h3>NVIDIA Isaac Lab</h3>
                <p>로봇 강화학습 연구를 위한 고성능 모듈식 프레임워크입니다. 핵심 개념과 고급 기법을 마스터하세요.</p>
                <Link className="button button--outline button--primary" to="/docs/isaac_lab/introduction_to_isaac_lab">
                  Isaac Lab 가이드
                </Link>
              </div>
              <div className={clsx('col col--4', styles.featureCard)}>
                <h3>ROS 2 Humble</h3>
                <p>로봇 애플리케이션 개발을 위한 표준 프레임워크입니다. Isaac Sim/Lab과의 연동 방법을 자세히 다룹니다.</p>
                <Link className="button button--outline button--primary" to="/docs/ros2_humble/ros2_basics_refresh">
                  ROS 2 가이드
                </Link>
              </div>
            </div>
          </div>
        </section>
        
        {/* Add more sections as needed, e.g., Featured Blog Posts, Call to Community Contribution */}

      </main>
    </Layout>
  );
}
