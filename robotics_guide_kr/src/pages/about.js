import React from 'react';
import Layout from '@theme/Layout';
import styles from './about.module.css'; // We can create this for custom styling if needed

export default function AboutPage() {
  return (
    <Layout
      title="소개 및 문의"
      description="로보틱스 시뮬레이션 마스터 가이드 웹사이트와 운영 목적, 그리고 문의 방법에 대해 알아보세요.">
      <div className="container margin-vert--xl">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1 className={styles.aboutTitle}>소개 (About Us)</h1>
            <p className={styles.aboutText}>
              이 웹사이트는 NVIDIA Isaac Sim, Isaac Lab, 그리고 ROS 2 Humble을 배우고 효과적으로 활용하고자 하는
              한국의 개발자, 연구자, 학생들을 위한 포괄적인 가이드와 튜토리얼을 제공하기 위해 만들어졌습니다.
              로보틱스 시뮬레이션과 AI 기반 로봇 개발의 세계는 빠르게 발전하고 있으며, 이 강력한 도구들을
              한국어로 쉽게 접근하고 깊이 있게 이해하는 데 도움이 되고자 합니다.
            </p>
            <p className={styles.aboutText}>
              본 가이드는 각 플랫폼의 기초부터 시작하여 고급 주제, 실제 적용 사례, 그리고 유용한 팁까지
              다양한 수준의 콘텐츠를 포함하는 것을 목표로 합니다. 로보틱스 분야에 처음 입문하는 분들부터
              숙련된 전문가들까지 모두에게 유용한 자료가 되기를 바랍니다.
            </p>
            
            <h2 className={styles.sectionTitle}>운영 목적</h2>
            <ul className={styles.purposeList}>
              <li>NVIDIA Isaac Sim, Isaac Lab, ROS 2 관련 최신 정보 및 기술 심층 분석 한국어 제공</li>
              <li>로보틱스 시뮬레이션 및 AI 개발 학습을 위한 실용적인 튜토리얼 및 예제 공유</li>
              <li>국내 로보틱스 개발자 커뮤니티의 지식 교류 및 기술 성장 지원</li>
              <li>Sim-to-Real 워크플로우 구축을 위한 아이디어와 경험 공유</li>
            </ul>

            <h2 className={styles.sectionTitle}>콘텐츠 기여 및 제안</h2>
            <p className={styles.aboutText}>
              이 웹사이트는 커뮤니티의 참여와 기여를 적극 환영합니다! 내용 추가, 수정 제안, 오타 지적,
              새로운 예제 프로젝트 아이디어 등 어떤 형태의 피드백이나 기여든 좋습니다.
              함께 만들어가는 유용한 지식 저장소가 되기를 기대합니다.
            </p>
            <p className={styles.aboutText}>
              기여나 제안은 본 웹사이트의 GitHub 저장소를 통해 Pull Request 또는 Issue를 생성하여 전달해주시면 감사하겠습니다.
              (GitHub 링크는 푸터 또는 네비게이션 바를 참조하세요.)
            </p>

            <h2 className={styles.sectionTitle}>문의하기 (Contact)</h2>
            <p className={styles.aboutText}>
              웹사이트 내용, 특정 가이드, 또는 기타 문의사항이 있으시면 아래 연락처로 연락 주시기 바랍니다.
            </p>
            <ul className={styles.contactList}>
              <li><strong>Email:</strong> [your-guide-email@example.com] (실제 이메일 주소로 교체 예정)</li>
              <li><strong>GitHub Issues:</strong> 웹사이트 저장소의 Issue 트래커 (버그 리포트, 콘텐츠 요청 등)</li>
              {/* <li><strong>Discord/Slack:</strong> (향후 커뮤니티 채널 개설 시 추가)</li> */}
            </ul>
            <p className={styles.aboutText}>
              여러분의 관심과 참여가 이 가이드를 더욱 풍부하고 유용하게 만듭니다. 감사합니다.
            </p>
          </div>
        </div>
      </div>
    </Layout>
  );
}
