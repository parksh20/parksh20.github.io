import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: '종합적인 가이드',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default, // Placeholder SVG
    description: (
      <>
        NVIDIA Isaac Sim, Isaac Lab, ROS 2 Humble의 기초부터 고급 활용법까지 모든 것을 다룹니다.
        체계적인 튜토리얼과 심층 분석을 통해 로보틱스 시뮬레이션 전문가로 거듭나세요.
      </>
    ),
  },
  {
    title: '실전 예제 프로젝트',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default, // Placeholder SVG
    description: (
      <>
        실제 로봇 개발에 적용할 수 있는 다양한 예제 프로젝트를 통해 학습한 내용을 바로 확인하고 응용력을 키울 수 있습니다. 
        단계별 가이드를 따라 직접 만들어보세요.
      </>
    ),
  },
  {
    title: '최신 기술 및 커뮤니티',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default, // Placeholder SVG
    description: (
      <>
        로보틱스 시뮬레이션 분야의 최신 기술 동향과 Isaac Sim/Lab의 업데이트 정보를 빠르게 접할 수 있습니다. 
        활발한 커뮤니티와 함께 성장하세요. (향후 포럼/Q&A 연동 구상)
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        {/* <Svg className={styles.featureSvg} role="img" /> */}
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
