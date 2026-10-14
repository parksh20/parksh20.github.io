#!/usr/bin/env python3
"""Builds index.html and work/*.html.

Run from anywhere:  python _build/build.py
Every visible string is written twice, English and Korean, through T().
"""
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EMAIL = 'parksh000124@gmail.com'
GH = 'https://github.com/parksh20'
SITE = 'https://parksh20.github.io/'


def T(en, ko, tag='span', cls='', attr=''):
    c = f' {cls}' if cls else ''
    return f'<{tag} class="en{c}"{attr}>{en}</{tag}><{tag} class="ko{c}" lang="ko"{attr}>{ko}</{tag}>'


ARROW = '<svg class="arrow" viewBox="0 0 20 20" width="16" height="16" aria-hidden="true"><path d="M4 10h11M11 5l5 5-5 5" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"/></svg>'
BACK = '<svg viewBox="0 0 20 20" width="14" height="14" aria-hidden="true"><path d="M16 10H5M9 5l-5 5 5 5" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"/></svg>'
MAIL = '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M3 6.5h18v11H3z" fill="none" stroke="currentColor" stroke-width="1.8"/><path d="m3.5 7 8.5 6.5L20.5 7" fill="none" stroke="currentColor" stroke-width="1.8"/></svg>'
COPY = '<svg viewBox="0 0 24 24" aria-hidden="true"><rect x="8" y="8" width="12" height="12" rx="2" fill="none" stroke="currentColor" stroke-width="1.8"/><path d="M16 8V5a1 1 0 0 0-1-1H5a1 1 0 0 0-1 1v10a1 1 0 0 0 1 1h3" fill="none" stroke="currentColor" stroke-width="1.8"/></svg>'
GHICON = '<svg viewBox="0 0 24 24" aria-hidden="true"><path fill="currentColor" d="M12 2a10 10 0 0 0-3.16 19.49c.5.09.68-.22.68-.48v-1.7c-2.78.6-3.37-1.34-3.37-1.34-.45-1.16-1.11-1.47-1.11-1.47-.91-.62.07-.6.07-.6 1 .07 1.53 1.03 1.53 1.03.9 1.52 2.34 1.08 2.91.83.09-.65.35-1.08.63-1.33-2.22-.25-4.55-1.11-4.55-4.94 0-1.09.39-1.98 1.03-2.68-.1-.25-.45-1.27.1-2.65 0 0 .84-.27 2.75 1.02a9.6 9.6 0 0 1 5 0c1.91-1.29 2.75-1.02 2.75-1.02.55 1.38.2 2.4.1 2.65.64.7 1.03 1.59 1.03 2.68 0 3.84-2.34 4.68-4.57 4.93.36.31.68.92.68 1.85v2.75c0 .27.18.58.69.48A10 10 0 0 0 12 2Z"/></svg>'
LOOP = '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M4 12a8 8 0 0 1 14-5.3M20 12a8 8 0 0 1-14 5.3" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"/><path d="M18 3v4h-4M6 21v-4h4" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"/></svg>'


# ---------------------------------------------------------------- layout

def head(title, desc, p, path):
    return f'''<!doctype html>
<html lang="en" data-lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title}</title>
<meta name="description" content="{desc}">
<meta name="author" content="Sunghyun Park">
<meta property="og:type" content="website">
<meta property="og:site_name" content="Sunghyun Park">
<meta property="og:title" content="{title}">
<meta property="og:description" content="{desc}">
<meta property="og:image" content="{SITE}assets/img/og.jpg">
<meta property="og:url" content="{SITE}{path}">
<meta name="twitter:card" content="summary_large_image">
<meta name="theme-color" content="#0b0d12">
<link rel="icon" href="{p}assets/img/favicon.svg" type="image/svg+xml">
<link rel="preconnect" href="https://cdn.jsdelivr.net" crossorigin>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://cdn.jsdelivr.net/gh/orioncactus/pretendard@v1.3.9/dist/web/variable/pretendardvariable-dynamic-subset.min.css">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Geist+Mono:wght@400;500;600&display=swap">
<link rel="stylesheet" href="{p}assets/css/site.css">
<script>
(function () {{
  var l = 'en';
  try {{
    var q = new URLSearchParams(location.search).get('lang'), s = null;
    try {{ s = localStorage.getItem('lang'); }} catch (e) {{}}
    l = q || s || ((navigator.language || '').toLowerCase().indexOf('ko') === 0 ? 'ko' : 'en');
  }} catch (e) {{}}
  if (l !== 'ko') l = 'en';
  document.documentElement.setAttribute('data-lang', l);
  document.documentElement.lang = l;
  document.documentElement.classList.add('js-reveal');
}})();
</script>
</head>
'''


def nav(p, home=False, dark=False):
    h = '' if home else f'{p}index.html'
    links = [('#work', 'Work', '작업'), ('#about', 'About', '소개'), ('#publications', 'Publications', '논문'), ('#contact', 'Contact', '연락')]
    a = ''.join(f'<a href="{h}{k}">{T(en, ko)}</a>' for k, en, ko in links)
    return f'''<a class="skip" href="#main">Skip to content</a>
<header class="nav{' on-dark' if dark else ''}" id="nav">
  <div class="wrap nav-in">
    <a class="brand" href="{h or '#top'}"><span class="mark">SP</span><span>Sunghyun Park</span></a>
    <nav class="nav-links" aria-label="Sections">{a}</nav>
    <div class="nav-right">
      <div class="lang" role="group" aria-label="Language"><button type="button" data-set-lang="en">EN</button><button type="button" data-set-lang="ko">KO</button></div>
      <a class="nav-cta" href="mailto:{EMAIL}">{T('Email', '이메일')}</a>
    </div>
  </div>
</header>
'''


def tail(p):
    return f'''<div class="lightbox" id="lightbox" hidden><button type="button" class="lb-close" aria-label="Close">×</button><img alt=""></div>
<div class="toast" id="toast" role="status" aria-live="polite"></div>
<script src="{p}assets/js/site.js" defer></script>
</body>
</html>
'''


def foot_line(p):
    return f'''<div class="foot">
      <span>© 2026 Sunghyun Park</span>
      <span><a href="mailto:{EMAIL}">{EMAIL}</a> &nbsp;/&nbsp; <a href="{GH}">GitHub</a></span>
      <span>{T('Last updated October 2026', '2026년 10월 갱신')}</span>
    </div>'''


# ---------------------------------------------------------------- pieces

def tags(*items):
    cls = {'real': ' tag-real', 'sim': ' tag-sim', 'red': ' tag-red', '': ''}
    return '<div class="tags">' + ''.join(f'<span class="tag{cls[k]}">{t}</span>' for t, k in items) + '</div>'


def video(src, poster, label, tg='', cap='', ratio='r169', preload='none'):
    c = f'<div class="media-cap">{cap}</div>' if cap else ''
    return (f'<div class="media {ratio}">{tg}<video class="autoplay" src="{src}" poster="{poster}" muted loop playsinline '
            f'preload="{preload}" aria-label="{label}"></video>{c}</div>')


def image(src, alt, tg='', ratio='r1610', cap=''):
    c = f'<div class="media-cap">{cap}</div>' if cap else ''
    return f'<div class="media {ratio}">{tg}<img src="{src}" alt="{alt}" loading="lazy">{c}</div>'


def zoomfig(src, w, h, alt, cap=''):
    c = f'<p class="cap-text">{cap}</p>' if cap else ''
    return (f'<figure><button class="zoom" type="button" data-zoom="{src}" aria-label="Enlarge figure"><div class="figframe">'
            f'<img src="{src}" width="{w}" height="{h}" loading="lazy" alt="{alt}"></div></button>'
            f'<p class="zoom-hint">{T("Click to enlarge", "눌러서 크게 보기")}</p>{c}</figure>')


def bars(title, meta, groups, xlabels, aria, cls=''):
    gl = ''.join(f'<i class="{"zero" if y == 0 else ""}" style="bottom:{y}%"><b>{y}</b></i>' for y in (100, 75, 50, 25, 0))
    cols = ''
    for g in groups:
        cols += '<div class="col">' + ''.join(f'<div class="bar {k}" style="--v:{v}"><b>{v:.1f}</b></div>' for k, v in g) + '</div>'
    xl = ''.join(f'<span>{x}</span>' for x in xlabels)
    return (f'<div class="chart {cls}" role="img" aria-label="{aria}"><p class="chart-title">{title} <span class="label">{meta}</span></p>'
            f'<div class="plot"><div class="gl">{gl}</div><div class="cols">{cols}</div></div><div class="xl">{xl}</div></div>')


def linechart(title, xlabels, series, aria):
    W, H, L, R, TP, B = 470, 300, 40, 14, 30, 52
    pw, ph = W - L - R, H - TP - B
    xs = [L + pw * (i + .5) / len(xlabels) for i in range(len(xlabels))]
    y = lambda v: TP + ph * (1 - v / 100)
    s = [f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="{aria}">']
    for g in (0, 25, 50, 75, 100):
        dash = '' if g == 0 else ' stroke-dasharray="3 4"'
        col = '#3b3f49' if g == 0 else '#dcd9d1'
        s.append(f'<line x1="{L}" x2="{W - R}" y1="{y(g):.1f}" y2="{y(g):.1f}" stroke="{col}" stroke-width="{1.5 if g == 0 else 1}"{dash}/>')
        s.append(f'<text x="{L - 10}" y="{y(g) + 4:.1f}" text-anchor="end" font-size="13" fill="#777b86">{g}</text>')
    for name, color, vals, above in series:
        pts = ' '.join(f'{x:.1f},{y(v):.1f}' for x, v in zip(xs, vals))
        s.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="3" stroke-linejoin="round" stroke-linecap="round"/>')
        for x, v in zip(xs, vals):
            s.append(f'<circle cx="{x:.1f}" cy="{y(v):.1f}" r="5" fill="#fff" stroke="{color}" stroke-width="2.5"/>')
            ty = y(v) - 12 if above else y(v) + 24
            s.append(f'<text x="{x:.1f}" y="{ty:.1f}" text-anchor="middle" font-size="15" font-weight="600" fill="{color}">{v}</text>')
    for x, lab in zip(xs, xlabels):
        s.append(f'<text x="{x:.1f}" y="{H - B + 26}" text-anchor="middle" font-size="13.5" fill="#3b3f49">{lab}</text>')
    s.append('</svg>')
    return f'<div class="chart linechart"><p class="chart-title">{title}</p>{"".join(s)}</div>'


def case_steps(rows):
    names = [('Symptom', '현상'), ('Analysis', '분석'), ('Fix', '조치'), ('Result', '결과')]
    out = ''.join(f'<div><p class="label">{T(*n)}</p><p>{T(en, ko)}</p></div>' for n, (en, ko) in zip(names, rows))
    return f'<div class="case"><div class="case-steps">{out}</div></div>'


def meta_grid(items):
    return '<div class="cs-meta">' + ''.join(f'<div><p class="label">{T(*k)}</p><p>{T(*v)}</p></div>' for k, v in items) + '</div>'


def sec(id_, num, en, ko, body):
    return f'<section class="cs-sec" id="{id_}"><p class="label">{num}</p><h2>{T(en, ko)}</h2>{body}</section>'


def toc(items):
    return '<nav class="toc" aria-label="On this page">' + ''.join(f'<a href="#{i}">{T(en, ko)}</a>' for i, en, ko in items) + '</nav>'


def next_link(href, en, ko, title, sub_en, sub_ko):
    return f'''<a class="next" href="{href}"><div class="wrap">
  <p class="label">{T('Next project', '다음 프로젝트')}</p>
  <h2>{title} {ARROW.replace('width="16" height="16"', 'width="56" height="56"')}</h2>
  <p>{T(sub_en, sub_ko)}</p>
</div></a>'''


def case_page(p, fname, title, desc, header, hero, sections, toc_items, nxt):
    html = head(title, desc, p, f'work/{fname}') + '<body>\n' + nav(p) + '<main id="main">\n'
    html += f'<header class="cs-head"><div class="wrap">{header}</div></header>\n'
    html += f'<div class="cs-hero"><div class="wrap">{hero}</div></div>\n'
    html += f'<div class="wrap"><div class="cs-body">{toc(toc_items)}<div>{"".join(sections)}</div></div></div>\n'
    html += '</main>\n' + nxt + f'<footer class="mini-foot"><div class="wrap">{foot_line(p)}</div></footer>\n' + tail(p)
    with open(os.path.join(ROOT, 'work', fname), 'w', encoding='utf-8', newline='\n') as f:
        f.write(html)


def cs_header(label, h1, small, lede):
    return (f'<a class="crumb" href="../index.html#work">{BACK} {T("All work", "전체 작업")}</a>'
            f'<p class="label">{label}</p><h1>{h1}<small>{small}</small></h1><p class="cs-lede">{lede}</p>')


# ================================================================ HOME

def build_home():
    p = ''
    v = 'assets/video/'
    im = 'assets/img/'
    hero = f'''
<section class="hero" id="top">
  <div class="wrap">
    <p class="status"><span class="dot"></span>{T('Graduating February 2027, open to full-time roles', '2027년 2월 졸업 예정, 채용 문의 환영')}</p>
    <div class="hero-name"><h1>Sunghyun Park</h1><span class="ko-name">박성현</span></div>
    <div class="hero-grid">
      <div>
        <p class="hero-role">{T('Robot learning for bimanual manipulation', '양팔 로봇 조작을 위한 Robot learning')}</p>
        <p class="hero-desc">{T('Policies trained only in simulation, run on a real two-arm robot with no additional real-robot training.', '시뮬레이션에서만 학습한 양팔 Policy를 실물 추가 학습 없이 실제 로봇에 전이')}</p>
        <div class="hero-cta">
          <a class="btn btn-light" href="#work">{T('View work', '작업 보기')} {ARROW}</a>
          <a class="btn btn-ghost-dark" href="mailto:{EMAIL}">{MAIL} {T('Email', '이메일')}</a>
        </div>
        <div class="hero-facts">
          <p><b>{T('Now', '현재')}</b><span>{T('M.S. student, Kyung Hee University', '경희대학교 대학원 석사과정')}</span></p>
          <p><b>{T('Robot', '로봇')}</b><span>UR3 × 2, Allegro Hand × 2</span></p>
          <p><b>{T('Tools', '도구')}</b><span>Isaac Lab, PyTorch, PPO, LeRobot</span></p>
        </div>
      </div>
      <div class="hero-media">{video(v + 'hero_pnp.mp4', v + 'hero_pnp.jpg', 'Real robot: TMAD student places a red can and a green can into the bin', tags(('Real robot', 'real'), ('Zero-shot', ''), ('2× speed', '')), T('TMAD student, Pick-and-Place', 'TMAD student, Pick-and-Place'), preload='metadata')}</div>
    </div>
    <div class="hero-stats">
      <div class="hstat"><p class="hstat-num red">72.0<small>%</small></p><p class="hstat-label">{T('Pick-and-Place, real robot', 'Pick-and-Place, 실물')}</p><p class="hstat-sub">Baseline 28.0%</p></div>
      <div class="hstat"><p class="hstat-num red">84.0<small>%</small></p><p class="hstat-label">{T('Handover, real robot', 'Handover, 실물')}</p><p class="hstat-sub">Baseline 24.0%</p></div>
      <div class="hstat"><p class="hstat-num">75</p><p class="hstat-label">{T('Real-robot trials per task', '작업별 실물 시행')}</p><p class="hstat-sub">3 runs × 25 trials</p></div>
      <div class="hstat hstat-note">{T('TMAD student compared with the single-arm DextrAH-RGB student ported to two arms. Zero-shot: no additional training on the real robot.', 'TMAD student와, 단일 팔용 DextrAH-RGB를 양팔로 이식한 Student의 비교. 실물 추가 학습 없음')}</div>
    </div>
  </div>
</section>'''

    tmad = f'''
<a class="wcard wcard-feature" href="work/tmad.html" data-reveal>
  <div class="wcard-media">
    <div class="pip-main">{video(v + 'real_handover.mp4', v + 'real_handover.jpg', 'Real robot Handover', tags(('Real robot', 'real'), ('Zero-shot', ''), ('2×', '')), T('Handover, robot camera', 'Handover, 로봇 카메라'))}</div>
    <div class="pip">{video(v + 'sim_handover.mp4', v + 'sim_handover.jpg', 'Simulation Handover', tags(('Simulation', 'sim'),))}</div>
  </div>
  <div class="wcard-body">
    <p class="label"><span class="num-tag">01</span><span class="sep">/</span>{T('Research, first author', '연구, 제1저자')}<span class="sep">/</span>2026</p>
    <h3>TMAD<small>Temporal Multi-Agent Distillation for Bimanual Long-Horizon Manipulation</small></h3>
    <p class="wdesc">{T('A teacher that reads object states in simulation, distilled into a student that sees two RGB images and joint states. Only the student runs on the real robot.', 'Simulation에서 물체 상태를 받는 Teacher를, 영상 두 장과 관절 값만 보는 Student로 Distillation. 실제 로봇에서는 Student만 실행')}</p>
    <div class="chips"><span class="chip hl">{T('ICCR 2026, under review', 'ICCR 2026 심사 중')}</span><span class="chip">{T('Patent application filed', '특허 출원')}</span><span class="chip">{T('M.S. thesis', '석사 학위논문')}</span></div>
    <div class="wmetrics">
      <div class="wmetric"><b>72.0%</b><span>{T('Pick-and-Place, real robot', 'Pick-and-Place, 실물')}</span><span class="mono">Baseline 28.0%</span></div>
      <div class="wmetric"><b>84.0%</b><span>{T('Handover, real robot', 'Handover, 실물')}</span><span class="mono">Baseline 24.0%</span></div>
    </div>
    <span class="wmore">{T('Read the case study', '자세히 보기')} {ARROW}</span>
  </div>
</a>'''

    frames = [('v_seq_1', 'start', 'green'), ('v_seq_3', 'green placed', 'red'), ('v_seq_4', 'grasp red', ''), ('v_seq_5', 'both placed', 'done')]
    strip = ''.join(f'<figure><span class="call{"" if c else " none"}">{c or "-"}</span><img src="{im}{f}.webp" alt="{lab}" loading="lazy"><figcaption>{lab}</figcaption></figure>' for f, lab, c in frames)
    vla = f'''
<a class="wcard wcard-col" href="work/vla.html" data-reveal>
  <div class="wcard-media vla-media"><div class="vla-vis">
    <p class="vla-quote"><span class="label">{T('Instruction', '지시문')}</span>"Put the green can on the pad first, then the red can."</p>
    <div class="vla-roles"><p><span class="who navy">VLM planner</span><span class="what">green</span></p><p><span class="who red">SmolVLA</span><span class="what">"put the green can on the pad"</span></p></div>
    <div class="vla-strip">{strip}</div>
  </div></div>
  <div class="wcard-body">
    <p class="label"><span class="num-tag">02</span><span class="sep">/</span>{T('Personal project', '개인 프로젝트')}<span class="sep">/</span>2026.10<span class="sep">/</span>Simulation</p>
    <h3>VLM planner + SmolVLA<small>{T('Language instructions for the same two-arm robot', '같은 양팔 로봇의 자연어 지시 수행')}</small></h3>
    <p class="wdesc">{T('A fine-tuned VLM picks the next skill, SmolVLA moves both arms. Grasp failures fixed with takeover DAgger and Residual RL distillation.', 'Fine-tuning한 VLM이 다음 skill을 고르고 SmolVLA가 두 팔을 움직임. Grasp 실패는 Takeover DAgger와 Residual RL distillation으로 해결')}</p>
    <div class="wmetrics"><div class="wmetric navy"><b><s>41.0</s>83.4%</b><span>{T('Task success rate, before and after the fixes (simulation)', 'Task success rate, 교정 전과 후(Simulation)')}</span></div></div>
    <span class="wmore">{T('Read the case study', '자세히 보기')} {ARROW}</span>
  </div>
</a>'''

    ur = f'''
<a class="wcard wcard-col" href="work/ur2026.html" data-reveal>
  <div class="wcard-media"><img src="{im}u_sweep_4.webp" alt="Bimanual sweeping in simulation and on the real robot" loading="lazy" style="object-position:left center"></div>
  <div class="wcard-body">
    <p class="label"><span class="num-tag">03</span><span class="sep">/</span>{T('Co-author', '공동저자')}<span class="sep">/</span>UR 2026</p>
    <h3>Upper-Humanoid Real2Sim2Real<small>{T('Perception, sweeping assets and the sweeping task in a team system', '팀 시스템에서 Perception, Sweeping asset, Sweeping task 담당')}</small></h3>
    <p class="wdesc">{T('A YOLO detector that starts FoundationPose tracking without a person, a CAD broom for Isaac Lab, and a bimanual sweeping task with stage-wise rewards.', '사람 개입 없이 FoundationPose 추적을 시작하는 YOLO detector, Isaac Lab용 CAD 빗자루, 단계별 Reward의 양팔 쓸기 작업')}</p>
    <div class="wmetrics"><div class="wmetric"><b>85.7%</b><span>{T('Bimanual Sweeping, real robot (team)', 'Bimanual Sweeping, 실물(팀 결과)')}</span></div><div class="wmetric"><b>82.8%</b><span>{T('Cup Handover, real robot (team)', 'Cup Handover, 실물(팀 결과)')}</span></div></div>
    <span class="wmore">{T('Read the case study', '자세히 보기')} {ARROW}</span>
  </div>
</a>'''

    assets = f'''
<a class="wcard wcard-wide" href="work/sim-assets.html" data-reveal>
  <div class="wcard-media">
    {image(im + 'a_real_sweep.webp', 'Real sweeping setup', tags(('Real', 'real'),), 'r169')}
    {video(v + 'sim_sweep.mp4', v + 'sim_sweep.jpg', 'Isaac Lab sweeping scene', tags(('Isaac Lab', 'sim'),))}
  </div>
  <div class="wcard-body">
    <p class="label"><span class="num-tag">04</span><span class="sep">/</span>Simulation assets<span class="sep">/</span>2025 ~ 2026</p>
    <h3>{T('Real tools, rebuilt for Isaac Lab', '실물 도구를 Isaac Lab으로')}<small>{T('Broom, dustpan, broom supporter, cans, photogrammetry', '빗자루, 쓰레받기, 서포터, 캔, Photogrammetry')}</small></h3>
    <p class="wdesc">{T('Broom and supporter modeled in CAD, a dustpan resized to the lab one, cans with real label textures. Appearance from photos, collision from separate colliders.', '빗자루와 서포터는 CAD로 직접 모델링, 쓰레받기는 실험실 크기로, 캔은 실물 라벨 텍스처. 겉모습은 실물 사진 기준, 충돌은 별도 Collider')}</p>
    <div class="chips"><span class="chip">Fusion</span><span class="chip">Blender</span><span class="chip">USD</span><span class="chip">Meshroom</span><span class="chip">Isaac Lab</span></div>
    <span class="wmore">{T('Read the case study', '자세히 보기')} {ARROW}</span>
  </div>
</a>'''

    work = f'''
<section class="sec" id="work">
  <div class="wrap">
    <header class="sec-head"><p class="label"><span class="num-tag">01</span><span class="sep">/</span>Work</p>
      <div class="sec-side"><h2>{T('Selected work', '주요 작업')}</h2><p class="sec-desc">{T('Four projects from my M.S., from simulation assets to a policy running on a real robot.', '석사 과정의 네 프로젝트. Simulation asset부터 실제 로봇에서 동작하는 Policy까지')}</p></div>
    </header>
    <div class="work-list">{tmad}<div class="work-two">{vla}{ur}</div>{assets}</div>
  </div>
</section>'''

    caps_data = [
        ('RL and distillation', '강화학습과 Distillation',
         [('Teacher training with PPO after BC initialization', 'BC 초기화 후 PPO로 Teacher 학습'), ('Distillation into a stereo RGB student', 'Stereo RGB Student로 Distillation'), ('Domain randomization design', 'Domain randomization 설계')],
         ['PyTorch', 'PPO', 'Isaac Lab']),
        ('VLA', 'VLA',
         [('Qwen3-VL-4B QLoRA fine-tuning', 'Qwen3-VL-4B QLoRA Fine-tuning'), ('Bimanual 44-D actions with SmolVLA', 'SmolVLA로 양팔 44-D Action 생성'), ('DAgger, Residual RL distillation', 'DAgger, Residual RL distillation')],
         ['LeRobot', 'QLoRA', 'PPO']),
        ('Simulation environments', 'Simulation 환경',
         [('Bimanual sweeping environment', 'Bimanual Sweeping 환경 구성'), ('Collision shapes, stage-wise rewards', 'Collision shape, 단계별 Reward'), ('CAD and photogrammetry 3D assets', 'CAD, Photogrammetry 3D asset')],
         ['USD', 'Meshroom', 'CAD']),
        ('Perception', 'Perception',
         [('YOLO detector for target objects', '대상 물체 YOLO detector 학습'), ('FoundationPose 6D tracking', 'FoundationPose 6D 추적 도입'), ('Tracking that starts from detections', '검출 결과로 추적 자동 시작')],
         ['YOLO', 'FoundationPose']),
    ]
    caps = ''
    for i, (en, ko, items, tools) in enumerate(caps_data, 1):
        lis = ''.join(f'<li>{T(a, b)}</li>' for a, b in items)
        tl = ''.join(f'<span class="chip">{t}</span>' for t in tools)
        caps += f'<div class="cap" data-reveal><p class="label">0{i}</p><h3>{T(en, ko)}</h3><ul>{lis}</ul><div class="tools">{tl}</div></div>'

    about = f'''
<section class="sec sec-alt" id="about">
  <div class="wrap">
    <header class="sec-head"><p class="label"><span class="num-tag">02</span><span class="sep">/</span>About</p>
      <div class="sec-side"><h2>{T('What I work on', '하는 일')}</h2>
      <p class="sec-desc">{T('M.S. student at Kyung Hee University, advised by Prof. Tae-Seong Kim. I train policies for a robot with two UR3 arms and Allegro hands in Isaac Lab and test them on the real robot. Before robotics, I took part in medical imaging collaborations and built the AI part of a food recognition app.', '경희대학교 대학원 석사과정, 지도교수 김태성. Isaac Lab에서 UR3 양팔과 Allegro Hand 로봇의 Policy를 학습하고 실제 로봇에서 검증합니다. 로봇 연구 전에는 의료영상 공동연구에 참여했고 음식 인식 앱의 AI 파트를 맡았습니다.')}</p></div>
    </header>
    <div class="caps">{caps}</div>
  </div>
</section>'''

    edu = [('2025.03 ~ 2027.02', ('M.S., Electronics and Information Convergence Engineering', '전자정보융합공학과 석사과정'), ('Kyung Hee University. Graduating February 2027.', '경희대학교 대학원, 2027년 2월 졸업 예정')),
           ('2019.03 ~ 2025.02', ('B.S., Biomedical Engineering', '생체의공학과 학사'), ('Kyung Hee University', '경희대학교'))]
    exp = [('2024.06 ~ 08', ('Intern, Innovation Development Team, Jeisys Medical', '현장실습, 제이시스메디칼 혁신개발팀'), ('Temperature measurement experiments on HIFU and RF devices. Automated clinical data processing with Excel VBA.', 'HIFU와 RF 장비 온도 측정 실험. VBA로 임상 데이터 정리 자동화')),
           ('2024.10', ('Encouragement Award, KHU Creative Design Competition', '교내 창의적종합설계 경진대회 장려상'), ('AI part of the diet app My GPTrainer, shared by two: about 60k synthetic images of 400 Korean dishes, three YOLO models compared, on-phone recognition with TFLite.', '음식 사진 식단 관리 앱 My GPTrainer의 AI 파트를 둘이 맡아, 한식 400종 합성 이미지 약 6만 장과 YOLO 3종 비교, TFLite로 휴대전화 인식')),
           ('2024.04 ~ 12', ('Team lead, ATLAS startup club', '창업동아리 ATLAS 팀장'), ('Founded the club with members recruited by interview. Split 10 members into two teams; both teams won awards at the campus competition.', '동아리원을 면접으로 모아 창업동아리를 만들고, 10명을 두 팀으로 나눠 교내 경진대회에서 두 팀 모두 수상')),
           (('Undergraduate', '학부'), ('Graduation research', '학부 졸업 연구'), ('Device that measures grip force and EMG together. Mechanics, circuit and firmware with ATmega128 and HX711.', '악력과 근전도(EMG)를 함께 재는 측정 장치. ATmega128, HX711로 기구, 회로, Firmware 제작'))]

    def tl(rows):
        out = ''
        for d, t, s in rows:
            d = T(*d) if isinstance(d, tuple) else d
            out += f'<div class="tl"><p class="tl-date">{d}</p><div><h4>{T(*t)}</h4><p>{T(*s)}</p></div></div>'
        return out

    background = f'''
<section class="sec" id="background">
  <div class="wrap">
    <header class="sec-head"><p class="label"><span class="num-tag">03</span><span class="sep">/</span>Background</p>
      <div class="sec-side"><h2>{T('Education and experience', '학력과 경험')}</h2></div></header>
    <div class="bg-grid">
      <div><p class="label" style="margin-bottom:14px">{T('Education', '학력')}</p>{tl(edu)}</div>
      <div><p class="label" style="margin-bottom:14px">{T('Experience', '경험')}</p>{tl(exp)}</div>
    </div>
  </div>
</section>'''

    pubs = [
        ('ICCR 2026', T('Under review', '심사 중'), True, 'TMAD: Temporal Multi-Agent Distillation for Bimanual Long-Horizon Manipulation', None,
         '<b>Sunghyun Park</b>, Ismael Espinoza, Gabin Kim, Joon Young Kim, Kangsan Lee, Tae-Seong Kim',
         T('First author. 8th International Conference on Control and Robotics.', '제1저자. 8th International Conference on Control and Robotics')),
        ('UR 2026', 'Work-in-Progress', False, 'An Upper-Humanoid Real2Sim2Real System for Bimanual Long-Horizon Manipulation', None,
         'Yong-Hyeok Choi, Ismael Espinoza, Kangsan Lee, <b>Sunghyun Park</b>, Joon Young Kim, Ji-Heon Oh, Won Hee Lee, Tae-Seong Kim',
         T('23rd International Conference on Ubiquitous Robots, Osaka, July 2026.', '23rd International Conference on Ubiquitous Robots, Osaka, 2026.07')),
        ('IEEE ICCE 2026', '', False, 'Enhancing RT-DETR with Reinforcement Learning for Intracranial Hemorrhage Detection and Classification in Brain CT Images', 'https://doi.org/10.1109/ICCE70609.2026.11658394',
         'Kangsan Lee, Hafiz Zahid Tufail, <b>Sunghyun Park</b>, Joon-Young Kim, Sana Ullah, Ji-Heon Oh, Dong-Luong Dinh, Won Hee Lee, Tae-Seong Kim',
         T('IEEE International Conference on Communications and Electronics. Early PPO implementation and dataset model training.', 'IEEE International Conference on Communications and Electronics. PPO 초기 구현과 데이터셋 모델 학습 참여')),
        ('MIDL 2026', 'Short paper', False, '3D Segmentation of Intracerebral Hemorrhage in Brain CT Using Enhanced UNet Transformer via Reinforcement Learning', 'https://openreview.net/forum?id=bdCXTRTQ29',
         'Sana Ullah, Gabin Kim, Dongwook Lee, Hafiz Zahid Tufail, Kangsan Lee, <b>Sunghyun Park</b>, Joon-Young Kim, Won Hee Lee, Tae-Seong Kim',
         T('Medical Imaging with Deep Learning. Early PPO implementation and dataset model training.', 'Medical Imaging with Deep Learning. PPO 초기 구현과 데이터셋 모델 학습 참여')),
    ]
    rows = ''
    for venue, status, hl, title, link, authors, note in pubs:
        st = f'<span class="chip{" hl" if hl else ""}">{status}</span>' if status else ''
        tt = f'<a href="{link}">{title} ↗</a>' if link else title
        rows += f'<article class="pub" data-reveal><div class="pub-venue"><b>{venue}</b>{st}</div><div><p class="pub-title">{tt}</p><p class="pub-authors">{authors}</p><p class="pub-note">{note}</p></div></article>'

    publications = f'''
<section class="sec sec-alt" id="publications">
  <div class="wrap">
    <header class="sec-head"><p class="label"><span class="num-tag">04</span><span class="sep">/</span>Publications</p>
      <div class="sec-side"><h2>{T('Papers and patent', '논문과 특허')}</h2></div></header>
    <div class="pubs">{rows}</div>
    <div class="pub-extra">
      <div class="xpub"><p class="label">{T('Patent', '특허')}</p><h4>{T('System and method for learning bimanual robot control policies based on temporal multi-agent distillation', '시간적 멀티 에이전트 증류에 기반한 양팔 로봇 제어 정책 학습 시스템 및 방법')}</h4><p>{T('Korean patent application, filed September 2026', '국내 특허 출원, 2026.09')}</p></div>
      <div class="xpub"><p class="label">{T('M.S. thesis', '석사 학위논문')}</p><h4>{T('Development of Vision-Based Robot Intelligence via Temporal Multi-Agent Distillation for Bimanual Long-Horizon Manipulation', '양팔 로봇의 Long-Horizon 사물 조작을 위한 시간-맥락 다중 에이전트 정책 증류 기반 시각 지능 개발')}</h4><p>{T('Kyung Hee University, February 2027 (expected)', '경희대학교, 2027.02 예정')}</p></div>
    </div>
  </div>
</section>'''

    contact = f'''
<section class="contact" id="contact">
  <div class="wrap">
    <p class="label"><span class="num-tag">05</span><span class="sep">/</span>Contact</p>
    <a class="contact-mail" href="mailto:{EMAIL}"><span>{EMAIL}</span></a>
    <p class="contact-text">{T('Next, I want to carry this path, from Isaac Lab assets through teacher-student distillation to real-robot validation, into research that adds simulation RL to VLA and world models, toward robot intelligence that works reliably in homes and factories.', 'Isaac Lab asset부터 Teacher-Student Distillation, 실물 검증까지 이어본 경험을 VLA와 World Model에 시뮬레이션 강화학습을 더하는 연구로 이어, 가정과 공장에서 높은 성공률로 쓰이는 로봇 지능을 만들고 싶습니다.')}</p>
    <div class="contact-cta">
      <a class="btn btn-light" href="mailto:{EMAIL}">{MAIL} {T('Send an email', '메일 보내기')}</a>
      <button class="btn btn-ghost-dark" type="button" data-copy="{EMAIL}">{COPY} {T('Copy address', '주소 복사')}</button>
      <a class="btn btn-ghost-dark" href="{GH}">{GHICON} GitHub</a>
    </div>
    {foot_line(p)}
  </div>
</section>'''

    html = head('Sunghyun Park | Robot Learning', 'Sunghyun Park (박성현). Bimanual robot policies trained in simulation and run on a real robot zero-shot: teacher-student distillation (TMAD), VLA, Isaac Lab.', p, '')
    html += '<body data-hero>\n' + nav(p, home=True, dark=True) + '<main id="main">' + hero + work + about + background + publications + '</main>\n' + contact + tail(p)
    with open(os.path.join(ROOT, 'index.html'), 'w', encoding='utf-8', newline='\n') as f:
        f.write(html)


# ================================================================ TMAD

def build_tmad():
    p = '../'
    v, im = '../assets/video/', '../assets/img/'
    header = cs_header(
        f'{T("Research, first author", "연구, 제1저자")}<span class="sep">/</span>2026',
        'TMAD', 'Temporal Multi-Agent Distillation for Bimanual Long-Horizon Manipulation',
        T('A teacher policy learns two-arm, long-horizon tasks in simulation with object states it reads from the simulator. TMAD distills it into a student that sees two RGB images and the joint states, and only the student runs on the real robot, with no additional training there.',
          'Teacher policy는 Simulation에서 Simulator가 주는 물체 상태로 양팔 Long-horizon 작업을 학습. TMAD는 이를 영상 두 장과 관절 값만 보는 Student로 Distillation하고, 실제 로봇에서는 실물 추가 학습 없이 Student만 실행'))
    header += meta_grid([
        (('Role', '역할'), ('First author: teacher training, student design, distillation, Pick-and-Place experiments, manuscript', '제1저자: Teacher 학습, Student 설계, Distillation 구현, Pick-and-Place 비교 실험, 원고')),
        (('Status', '상태'), ('ICCR 2026, under review. Patent application filed. M.S. thesis.', 'ICCR 2026 심사 중, 특허 출원, 석사 학위논문')),
        (('Robot', '로봇'), ('UR3 × 2, Allegro Hand × 2, two RealSense D415 (RGB only)', 'UR3 × 2, Allegro Hand × 2, RealSense D415 두 대(RGB만 사용)')),
        (('Stack', '도구'), ('Isaac Lab, PyTorch, PPO, online distillation, ResNet-18, Transformer', 'Isaac Lab, PyTorch, PPO, Online distillation, ResNet-18, Transformer')),
    ])
    hero = f'''<div class="cs-dark"><div class="cs-hero-grid">
  {video(v + 'sim_pnp.mp4', v + 'sim_pnp.jpg', 'Pick-and-Place in Isaac Lab', tags(('Simulation', 'sim'), ('1×', '')), preload='metadata')}
  {video(v + 'real_pnp.mp4', v + 'real_pnp.jpg', 'Pick-and-Place on the real robot', tags(('Real robot', 'real'), ('Zero-shot', ''), ('2×', '')), preload='metadata')}
</div></div>
<p class="cap-text">{T('TMAD student on Pick-and-Place, in Isaac Lab (left) and on the real robot (right).', 'Pick-and-Place의 TMAD student, Isaac Lab(왼쪽)과 실물(오른쪽)')}</p>'''

    s1 = f'''<p>{T('In simulation a policy can read object position and pose straight from the simulator. A real robot cannot: it has to estimate them from cameras, usually with a 6D pose estimator, and the estimation error and latency go straight into the policy.',
                  'Simulation에서는 물체 위치와 자세를 Simulator가 직접 주지만, 실제 로봇에서는 카메라와 6D pose estimator로 추정해야 합니다. 이때 추정 오차와 지연이 Policy로 그대로 전달됩니다.')}</p>
<div class="block compare">
  <div class="ccard">{image(im + 't_par_envs.webp', 'Parallel simulation environments', tags(('Simulation', 'sim'),))}<div class="ccard-body"><p class="label">Simulation</p><h3>{T('Object state is given', '물체 상태가 주어짐')}</h3><ul class="ticks"><li>{T('The simulator gives object position and pose directly', '물체 위치와 자세를 Simulator가 직접 제공')}</li><li>{T('A policy can learn the task without vision', 'Vision 없이도 Policy 학습 가능')}</li></ul></div></div>
  <div class="ccard">{image(im + 't_real_robot.webp', 'Real robot with two UR3 arms and Allegro hands', tags(('Real robot', 'real'),))}<div class="ccard-body"><p class="label">Real robot</p><h3>{T('Object state has to be estimated', '물체 상태를 추정해야 함')}</h3><ul class="ticks warn"><li>{T('Object pose estimated from cameras with a 6D pose estimator', '물체 자세를 카메라와 6D pose estimator로 추정')}</li><li>{T('Estimation error and latency pass into the policy', '추정 오차와 지연이 Policy로 전달')}</li></ul></div></div>
</div>
<div class="callout"><p class="label">{T('Approach', '제안')}</p><p>{T('Use the teacher only for training. On the robot, run a student that sees two RGB images and the joint states.', 'Teacher는 학습에만 쓰고, 실행은 영상 두 장과 관절 값만 보는 Student가 맡는다')}<small>{T('DextrAH-RGB distilled a state-based teacher into a stereo RGB student for one arm and a short grasping task. TMAD extends this to two arms and long-horizon tasks: the two arms are two agent tokens (multi-agent), and the student keeps a 32-step history (temporal).', 'DextrAH-RGB는 상태 기반 Teacher를 Stereo RGB Student로 Distillation했지만 한 팔, 짧은 잡기 작업. TMAD는 이를 양팔 Long-horizon 작업으로 확장. 양팔을 두 Agent token으로 다루고(Multi-Agent), Student에 최근 32 step 이력(Temporal)을 더함')}</small></p></div>'''

    s2 = f'''<div class="task-cards">
  <div class="ccard">{image(im + 't_real_pnp.webp', 'Pick-and-Place on the real robot', tags(('Real robot', 'real'),), 'r43')}<div class="ccard-body"><p class="label">Task 1</p><h3>Pick-and-Place</h3><p class="prose">{T('Each arm picks up a can and puts it in the bin in the middle.', '두 팔이 각각 캔을 집어 가운데 상자에 넣기')}</p><ol class="subgoals"><li>Grasp L</li><li>Place L</li><li>Grasp R</li><li>Place R</li><li>Return</li></ol></div></div>
  <div class="ccard">{image(im + 't_real_ho.webp', 'Handover on the real robot', tags(('Real robot', 'real'),), 'r43')}<div class="ccard-body"><p class="label">Task 2</p><h3>Handover</h3><p class="prose">{T('One arm picks up a tube and passes it to the other arm, which places it.', '한 팔이 집은 튜브를 다른 팔에 넘겨 놓기')}</p><ol class="subgoals"><li>Grasp</li><li>Transfer pose</li><li>Hand over</li><li>Place</li><li>Return</li></ol></div></div>
</div>
<p class="cap-text">{T('A trial counts as a success only if all five subgoals are completed.', 'Subgoal 5개를 모두 마친 Trial만 성공')}</p>'''

    s3 = f'''<div class="steps3">
  <div class="step s1">{image(im + 't_par_envs.webp', '500 parallel environments', tags(('Simulation', 'sim'),))}<div class="step-body"><span class="step-num">1</span><h3>{T('Teacher learns the task', 'Teacher가 작업 학습')}</h3><ul><li>{T('500 parallel environments', '병렬 환경 500개')}</li><li>{T('Input: object pose, goal, joints', '입력: 물체 자세, 목표, 관절 값')}</li><li>{T('BC on 300 demos, then PPO', 'BC 300 demos 뒤 PPO')}</li></ul><p class="formula">π<sub>T</sub>(a<sub>t</sub> | O<sub>T,t</sub>)</p></div></div>
  <div class="step s2"><div class="media r1610"><div class="aug-pair"><figure><img src="{im}t_simcam_l.webp" alt="Rendered camera image" loading="lazy">{tags(('Rendered', 'sim'),)}</figure><figure><img src="{im}t_aug9.webp" alt="Randomized camera image" loading="lazy">{tags(('Randomized', 'red'),)}</figure></div></div><div class="step-body"><span class="step-num">2</span><h3>{T('Student copies the teacher', 'Student가 Teacher를 따라 학습')}</h3><ul><li>{T('Rendered stereo images with color and camera randomization', '색과 카메라 Randomization을 거친 렌더링 Stereo 영상')}</li><li>{T("The teacher stays fixed; its actions are the labels", 'Teacher는 고정, Teacher의 Action이 정답')}</li><li>{T('Online distillation: control moves from teacher to student as β goes from 1 to 0', 'Online distillation: β가 1에서 0으로 줄며 실행이 Teacher에서 Student로')}</li></ul><p class="formula">L = L<sub>a</sub> + λ<sub>pose</sub> L<sub>pose</sub></p></div></div>
  <div class="step s3">{image(im + 't_real_robot.webp', 'Real robot', tags(('Real robot', 'real'), ('Zero-shot', '')))}<div class="step-body"><span class="step-num">3</span><h3>{T('Deploy only the student', '실물에는 Student만')}</h3><ul><li>{T('No 6D pose estimator', '6D pose estimator 없음')}</li><li>{T('No real-robot training data', '실물 학습 데이터 없음')}</li><li>{T('Two RealSense D415, RGB only', 'RealSense D415 두 대, RGB만')}</li></ul><p class="formula">a<sub>t</sub> = μ<sub>S</sub>(O<sub>S,t</sub>)</p></div></div>
</div>'''

    Y, N = '<span class="yes">✓</span>', '<span class="no">✕</span>'
    rows = [(('Object pose', '물체 자세'), Y, N), (('Goal position', '목표 위치'), Y, N), (('Joints and grip', '관절 값과 그립'), Y, Y),
            (('Two RGB images, 424 × 240', 'RGB 영상 두 장, 424 × 240'), N, Y), (('History', '이력'), '<span class="val">1 step</span>', '<span class="val red">32 steps</span>')]
    trs = ''.join(f'<tr><td>{T(*n)}</td><td class="c">{a}</td><td class="c">{b}</td></tr>' for n, a, b in rows)

    def node(cls, h, d):
        return f'<div class="fnode {cls}"><h4>{h}</h4><p>{d}</p></div>'

    fa = ''
    flow = (f'<div class="fnode in"><h4>{T('Two RGB images', 'RGB 영상 두 장')}</h4><div class="in-imgs"><img src="{im}t_simcam_l.webp" alt="left camera" loading="lazy"><img src="{im}t_simcam_r.webp" alt="right camera" loading="lazy"></div><p>{T('left and right, 424 × 240, plus joints', '왼쪽, 오른쪽 424 × 240과 관절 값')}</p></div>' + fa +
            node('add', 'ResNet-18 encoder', T('×2, shared, ImageNet pretrained', '×2 공유, ImageNet 사전학습')) + fa +
            node('add', 'Stereo cross-attention', T('fuses the left and right views', '왼쪽과 오른쪽 영상 융합')) + fa +
            node('', 'Arm tokens', '<span class="mono">R [z; q<sub>R</sub>]<br>L [z; q<sub>L</sub>]</span>') + fa +
            node('', 'Observation encoder', T('self-attention between the two arms', '두 팔 사이 Self-attention')) + fa +
            node('add', 'Causal temporal transformer', T('attends to the past 32 steps', '최근 32 step을 봄')) + fa +
            node('', 'Autoregressive decoder', T('left arm given the right arm', '오른팔 Action을 보고 왼팔')) + fa +
            node('out', '14-D action', T('(μ<sub>S</sub>, σ<sub>S</sub>), 7 per arm', '(μ<sub>S</sub>, σ<sub>S</sub>), 팔마다 7')))
    s4 = f'''<p>{T("The student keeps the teacher's two-arm structure and adds two things: vision, and a memory of the last 32 steps.", 'Student는 Teacher의 양팔 구조를 그대로 두고 두 가지를 더함: 영상 입력, 최근 32 step의 기억')}</p>
<div class="block"><table class="io"><thead><tr><th>{T('Policy input', 'Policy 입력')}</th><th class="c">Teacher</th><th class="c">Student</th></tr></thead><tbody>{trs}</tbody></table></div>
<div class="block flow-wrap"><div class="flow">{flow}</div>
<div class="flow-aux"><span>{T('Object pose head: auxiliary loss, training only', 'Object pose head: 보조 손실, 학습 때만 사용')}</span></div>
<ul class="legend"><li><i class="sw"></i>{T('Kept from the teacher', 'Teacher에서 그대로')}</li><li><i class="sw add"></i>{T('Added for vision and history', '영상과 이력을 위해 추가')}</li></ul></div>
<p class="cap-text">{T('At deployment the policy gets no object pose; the object pose head only adds a loss during training.', '실행할 때 물체 자세 입력은 없음. Object pose head는 학습 때 손실에만 쓰임')}</p>'''

    roles = [(('Teacher training', 'Teacher 학습'), ('Trained the Pick-and-Place teacher. When training collapsed, logged KL and action std to trace the cause.', 'Pick-and-Place Teacher 학습. 학습 붕괴 때 KL과 Action std를 추가 기록해 원인 추적')),
             (('Student design', 'Student 설계'), ('Added stereo vision and the 32-step temporal transformer to the two-arm structure.', '양팔 구조에 Stereo vision과 32 step Temporal transformer 추가')),
             (('Distillation', 'Distillation 구현'), ('Modified the single-arm DextrAH-RGB public code for two-arm inputs and outputs.', '단일 팔용 DextrAH-RGB 공개 코드를 양팔 입출력으로 수정')),
             (('Experiments and paper', '비교 실험과 원고'), ('Ran the Pick-and-Place comparison experiments and wrote the manuscript as first author.', 'Pick-and-Place 비교 실험, 제1저자 원고'))]
    s5 = '<div class="roles">' + ''.join(f'<div class="role"><p class="label">0{i}</p><h3>{T(*h)}</h3><p>{T(*d)}</p></div>' for i, (h, d) in enumerate(roles, 1)) + '</div>'

    sim = bars('Simulation', '5 runs × 100 trials', [[('t', 91.2), ('b', 83.0), ('s', 86.0)], [('t', 88.6), ('b', 76.2), ('s', 84.8)]], ['Pick-and-Place', 'Handover'],
               'Simulation success rates. Pick-and-Place: teacher 91.2, baseline 83.0, TMAD 86.0. Handover: teacher 88.6, baseline 76.2, TMAD 84.8.')
    real = bars('Real robot', '3 runs × 25 trials, zero-shot', [[('b', 28.0), ('s', 72.0)], [('b', 24.0), ('s', 84.0)]], ['Pick-and-Place', 'Handover'],
                'Real robot success rates. Pick-and-Place: baseline 28.0, TMAD 72.0. Handover: baseline 24.0, TMAD 84.0.')
    gap = f'''<div class="gapcard"><p class="label">{T('TMAD minus baseline, pp', 'TMAD와 Baseline의 차이, %p')}</p>
<div><p class="gapname">Pick-and-Place</p><p class="gaprow"><span>Simulation</span><b>+3.0</b></p><p class="gaprow real"><span>Real robot</span><b>+44.0</b></p></div>
<div><p class="gapname">Handover</p><p class="gaprow"><span>Simulation</span><b>+8.6</b></p><p class="gaprow real"><span>Real robot</span><b>+60.0</b></p></div></div>'''
    pnp_l = linechart(T('Pick-and-Place, per subgoal', 'Pick-and-Place, Subgoal별'), ['Grasp L', 'Place L', 'Grasp R', 'Place R', 'Return'],
                      [('Baseline', '#9a9eab', [64, 64, 68, 36, 28], False), ('TMAD', '#a3201b', [88, 84, 88, 76, 72], True)],
                      'Real robot Pick-and-Place success per subgoal. TMAD 88, 84, 88, 76, 72. Baseline 64, 64, 68, 36, 28.')
    ho_l = linechart(T('Handover, per subgoal', 'Handover, Subgoal별'), ['Grasp', 'Transfer', 'Hand over', 'Place', 'Return'],
                     [('Baseline', '#9a9eab', [80, 76, 60, 28, 24], False), ('TMAD', '#a3201b', [88, 84, 84, 80, 80], True)],
                     'Real robot Handover success per subgoal. TMAD 88, 84, 84, 80, 80. Baseline 80, 76, 60, 28, 24.')
    s6 = f'''<p>{T('The two students were close in simulation. On the real robot they were far apart.', 'Simulation에서는 비슷했던 두 Student가 실제 로봇에서는 크게 갈림')}</p>
<ul class="legend block"><li><i class="sw t"></i>{T('Teacher (simulation only)', 'Teacher (Simulation만)')}</li><li><i class="sw b"></i>{T('Baseline student (DextrAH-RGB, ported to two arms)', 'Baseline student (양팔로 이식한 DextrAH-RGB)')}</li><li><i class="sw s"></i>TMAD student (ours)</li></ul>
<div class="block charts" style="margin-top:16px">{sim}{real}{gap}</div>
<p class="cap-text">{T('Mean success rate (%), from Table I of the paper. Kept the same for both students: teacher, inputs, vision encoder, action space, randomization and training length. The teacher needs object states from the simulator, so it does not run on the real robot.', '평균 성공률(%), 원고 Table I. 두 Student에 같게 둔 조건: Teacher, 입력, Vision encoder, Action space, Randomization, 학습 길이. Teacher는 Simulator의 물체 상태가 필요해 실물에서는 실행하지 않음')}</p>
<div class="lines">{pnp_l}{ho_l}</div>
<ul class="legend"><li><i class="sw s"></i>TMAD student</li><li><i class="sw b"></i>Baseline student</li></ul>
<p class="cap-text">{T('Real robot, success rate (%) at each subgoal. A separate 25-trial evaluation per task (paper Fig. 5).', '실물, Subgoal별 성공률(%). 작업마다 별도 25 trials(원고 Fig. 5)')}</p>'''

    seq_p = [('0:17', 'Grasp the red can', '빨간 캔 잡기'), ('0:22', 'Into the bin', '상자에 넣기'), ('0:28', 'Grasp the green can', '초록 캔 잡기'), ('0:35', 'Both cans done', '두 캔 완료')]
    seq_h = [('0:04', 'Pick up the tube', '튜브 잡아 들기'), ('0:08', 'Transfer pose', '건네기 자세'), ('0:12', 'Hand over', '두 손으로 넘김'), ('0:17', 'Place and return', '놓고 복귀')]

    def strip(prefix, seq):
        return '<div class="strip">' + ''.join(f'<figure><img src="{im}{prefix}_{i}.webp" alt="{en}" loading="lazy"><span class="t">{t}</span><figcaption><b>{i}</b>{T(en, ko)}</figcaption></figure>' for i, (t, en, ko) in enumerate(seq, 1)) + '</div>'

    s7 = f'''<div class="vgrid">
  <figure>{video(v + 'real_handover.mp4', v + 'real_handover.jpg', 'Real robot Handover from the robot camera', tags(('Real robot', 'real'), ('2×', '')))}<figcaption>{T("Handover, the robot's left stereo camera", 'Handover, 로봇 Stereo 카메라 왼쪽 영상')}</figcaption></figure>
  <figure>{video(v + 'sim_handover.mp4', v + 'sim_handover.jpg', 'Handover in Isaac Lab', tags(('Simulation', 'sim'), ('1×', '')))}<figcaption>{T('Handover in Isaac Lab', 'Handover, Isaac Lab')}</figcaption></figure>
</div>
<div class="strip-row"><h3>Pick-and-Place <span class="label">{T('third-person video, 35 s', '3인칭 원본 영상 35초')}</span></h3>{strip('t_seq_pnp', seq_p)}</div>
<div class="strip-row"><h3>Handover <span class="label">{T('robot camera, 17 s', '로봇 카메라 원본 17초')}</span></h3>{strip('t_seq_ho', seq_h)}</div>
<p class="cap-text">{T('One successful trial each, from start to finish. Times are positions in the original video.', '성공 시행 하나를 처음부터 끝까지. 시각은 원본 영상 기준')}</p>'''

    s8 = case_steps([
        ('The student barely moved the first time it ran on the real robot.', '처음 실물에 올린 Student가 거의 움직이지 않음'),
        ('Compared real and simulated images and inputs, then reproduced the same symptom in a simulation with added noise.', '실물과 Simulation의 영상, 입력값을 비교하고 Noise를 넣은 Simulation에서 같은 증상 재현'),
        ('Physics randomization for the teacher, image color and camera pose randomization for the student, then retraining.', 'Teacher에 Physics randomization, Student에 영상 색과 카메라 위치 Randomization을 더해 재학습'),
        ('With changes made only in simulation, the policy started working on the real robot.', '실물 데이터 없이 Simulation만 고쳐 실물에서 Policy가 동작하기 시작')])

    s9 = f'''<p>{T('In the paper, each Allegro hand moved with one grip command. In the follow-up, the policy sets all 16 finger joints of each hand directly and predicts four steps of actions per query.', '논문에서는 Allegro 손이 그립 명령 하나로 움직였음. 후속 연구에서는 Policy가 손마다 16관절을 모두 직접 정하고, 한 번의 추론으로 4 step의 Action을 예측')}</p>
<div class="block tiles">
  <div class="tile gray"><p class="label">{T('Paper', '논문')}</p><b>7-D</b><span>{T('per arm: 6 arm joints + 1 grip command', '팔마다: 팔 관절 6 + 그립 명령 1')}</span></div>
  <div class="tile"><p class="label">{T('Follow-up', '후속')}</p><b>22-D</b><span>{T('per arm: 6 arm + 16 finger joints, 4-step action chunk', '팔마다: 팔 6 + 손가락 16관절, 4 step Action chunk')}</span></div>
  <div class="tile navy"><p class="label">{T('Simulation, Pick-and-Place', 'Simulation, Pick-and-Place')}</p><b>94.7%</b><span>{T('student, 142 / 150 (teacher 99.3%, 993 / 1,000)', 'Student, 142 / 150 (Teacher 99.3%, 993 / 1,000)')}</span></div>
</div>
<div class="block">{zoomfig(im + 'tmad_followup.webp', 2400, 1014, 'Follow-up architecture: teacher with entity tokens, student with stereo RGB and object tokens, shared bimanual policy with 4-step action chunks.', T('Follow-up structure: (a) teacher with six entity tokens, (b) stereo RGB student with object tokens from color heatmaps, (c) shared bimanual policy.', '후속 구조: (a) 6개 Entity token Teacher (b) 색 Heatmap에서 Object token을 만드는 Stereo RGB Student (c) 공유 양팔 Policy'))}</div>'''

    s10 = f'''<div class="figs2">
  {zoomfig(im + 'tmad_pipeline.webp', 2400, 698, 'TMAD pipeline figure', T('Pipeline, in the style of the paper figure', '학습 파이프라인, 논문 그림 양식'))}
  {zoomfig(im + 'tmad_student.webp', 2400, 985, 'TMAD student policy figure', T('Student policy, with the real-robot input at deployment', 'Student policy 구조와 실물 실행 입력'))}
</div>'''

    sections = [sec('problem', '01', 'Problem', '문제', s1), sec('tasks', '02', 'Two long-horizon tasks', '두 Long-horizon 작업', s2),
                sec('approach', '03', 'Approach', '방법', s3), sec('student', '04', 'Student policy', 'Student policy 구조', s4),
                sec('role', '05', 'My part', '내 담당', s5), sec('results', '06', 'Results', '결과', s6),
                sec('real', '07', 'On the real robot', '실제 로봇', s7), sec('debug', '08', 'Problem solving: the first real-robot run', '문제 해결: 첫 실물 실행', s8),
                sec('followup', '09', 'Follow-up: all 16 finger joints', '후속 연구: 손가락 16관절 직접 제어', s9), sec('figures', '10', 'Detailed figures', '상세 그림', s10)]
    toc_items = [('problem', '01 Problem', '01 문제'), ('tasks', '02 Tasks', '02 작업'), ('approach', '03 Approach', '03 방법'), ('student', '04 Student policy', '04 Student 구조'),
                 ('role', '05 My part', '05 내 담당'), ('results', '06 Results', '06 결과'), ('real', '07 Real robot', '07 실제 로봇'), ('debug', '08 Problem solving', '08 문제 해결'),
                 ('followup', '09 Follow-up', '09 후속 연구'), ('figures', '10 Figures', '10 상세 그림')]
    nxt = next_link('vla.html', '', '', 'VLM planner + SmolVLA', 'Language instructions for the same two-arm robot, in simulation', '같은 양팔 로봇의 자연어 지시 수행, Simulation')
    case_page(p, 'tmad.html', 'TMAD | Sunghyun Park', 'TMAD: a vision-free teacher distilled into a stereo RGB student for a two-arm robot. 72.0% and 84.0% on the real robot, zero-shot.', header, hero, sections, toc_items, nxt)


# ================================================================ VLA

def build_vla():
    p = '../'
    im = '../assets/img/'
    header = cs_header(
        f'{T("Personal project", "개인 프로젝트")}<span class="sep">/</span>2026.10<span class="sep">/</span>Simulation',
        'VLM planner + SmolVLA', T('Language-instructed bimanual manipulation in Isaac Lab', 'Isaac Lab 양팔 로봇의 자연어 지시 수행'),
        T('The same two-arm robot, now driven by language. A fine-tuned VLM reads the scene and the instruction and picks the next skill; SmolVLA turns that skill into motion for both arms. When grasping turned out to be the bottleneck, I fixed it with data: takeover DAgger, then Residual RL successes distilled back into the VLA.',
          '같은 양팔 로봇을 자연어 지시로 움직이는 프로젝트. Fine-tuning한 VLM이 장면과 지시문을 보고 다음 skill을 고르고, SmolVLA가 그 skill을 두 팔의 동작으로 바꿈. 병목이 Grasp로 드러난 뒤에는 Takeover DAgger와 Residual RL 성공 Trajectory를 VLA에 Distillation해 데이터로 해결'))
    header += meta_grid([
        (('Role', '역할'), ('Personal project', '개인 프로젝트')),
        (('Setting', '환경'), ('Isaac Lab simulation, UR3 × 2 + Allegro Hand × 2', 'Isaac Lab Simulation, UR3 × 2 + Allegro Hand × 2')),
        (('Models', '모델'), ('Qwen3-VL-4B with QLoRA, SmolVLA 450M', 'Qwen3-VL-4B (QLoRA), SmolVLA 450M')),
        (('Result', '결과'), ('Task success rate from 41.0% to 83.4%', 'Task success rate 41.0%에서 83.4%로')),
    ])
    frames = [('v_seq_1', 'start', 'green'), ('v_seq_3', 'green placed', 'red'), ('v_seq_4', 'grasp red', ''), ('v_seq_5', 'both placed', 'done')]
    strip = ''.join(f'<figure><span class="call{"" if c else " none"}">{c or "-"}</span><img src="{im}{f}.webp" alt="{lab}" loading="lazy"><figcaption>{lab}</figcaption></figure>' for f, lab, c in frames)
    hero = f'''<div class="cs-dark" style="padding:0;overflow:hidden"><div class="vla-vis" style="padding:36px">
  <p class="vla-quote" style="font-size:clamp(20px,2.4vw,30px)"><span class="label">{T('Instruction', '지시문')}</span>"Put the green can on the pad first, then the red can."</p>
  <div class="vla-strip">{strip}</div>
</div></div>
<p class="cap-text">{T('One episode, closed loop. The blue labels are planner calls; after each skill the arms return home and the planner looks again.', '한 에피소드, 폐루프. 파란 라벨은 planner 호출, skill마다 팔이 홈으로 돌아오면 planner가 다시 봄')}</p>'''

    sk = lambda on, s, cls='': f'<span class="skill{" on" if on else ""}{cls}">{s}</span>'
    s1 = f'''<div class="sys">
  <div class="snode">{image(im + 'v_seq_1.webp', 'Front camera view', tags(('Front camera', ''),), 'r1610')}<div class="snode-body"><p class="label">Instruction</p><p class="quote">"Put the green can on the pad first, then the red can."</p></div></div>
  <div class="snode navy"><div class="snode-body"><p class="label">VLM planner</p><h3>Qwen3-VL-4B</h3><p>{T('4-bit QLoRA fine-tuning on 21,216 samples from 1,632 scenes × 13 instruction types', '1,632 장면 × 13 지시 유형에서 만든 21,216 샘플로 4-bit QLoRA Fine-tuning')}</p><p><b>{T('Answers the next skill:', '다음 skill을 답함:')}</b></p><div class="skills">{sk(True, 'green')}{sk(False, 'red')}{sk(False, 'done')}{sk(False, 'stop', ' stop')}</div></div></div>
  <div class="snode red"><div class="snode-body"><p class="label">VLA executor</p><h3>SmolVLA 450M</h3><p>{T('Input: the skill as text ("put the green can on the pad"), the camera image and the joint state of both arms', '입력: skill 문장("put the green can on the pad"), 카메라 영상, 두 팔의 관절 상태')}</p><p>{T('Trained on 1,200 scripted demos (210k frames)', 'Scripted demo 1,200개(210k frames)로 학습')}</p><p><b>{T('Output: 44-D action chunk, 50 steps', '출력: 44-D Action chunk, 50 steps')}</b></p></div></div>
  <div class="snode">{image(im + 'v_seq_3.webp', 'Isaac Lab scene', tags(('Isaac Lab', 'sim'),), 'r1610')}<div class="snode-body"><p class="label">Robot</p><h3>UR3 × 2 + Allegro Hand × 2</h3><p>{T('Per arm: 6 arm + 16 finger joints', '팔마다: 팔 6 + 손가락 16관절')}</p></div></div>
</div>
<p class="loop-note">{LOOP}{T('After each skill the arms return home and the planner looks again.', 'skill이 끝날 때마다 팔이 홈으로 돌아오고 planner가 다시 봄')}</p>'''

    exs = [('v_ex_1', 'Leave the red can where it is and put the other can on the pad.', 'green', ''), ('v_ex_2', 'Put the green can on the pad first, then the red can.', 'red', ''), ('v_ex_3', 'Put both cans on the pad.', 'stop', ' stop')]
    exh = ''.join(f'<div class="ex"><img src="{im}{f}.webp" alt="Scene" loading="lazy"><div class="ex-body"><p class="quote">"{q}"</p><p class="ex-ans">answer {sk(True if c == "" else False, a, c)}</p></div></div>' for f, q, a, c in exs)
    s2 = f'''<p>{T('The planner has to read the scene and the instruction together: the same words can need different answers depending on where the cans are.', 'Planner는 장면과 지시문을 함께 읽어야 함. 같은 지시라도 캔이 어디 있느냐에 따라 답이 달라짐')}</p>
<div class="block examples">{exh}</div>
<div class="block tiles">
  <div class="tile"><p class="label">{T('Phrasing seen in training', '학습에 쓴 표현')}</p><div class="ba"><s>40.1</s><b>99.5%</b></div><span>{T('zero-shot Qwen3-VL-4B, then after QLoRA', 'Zero-shot Qwen3-VL-4B, QLoRA 뒤')}</span></div>
  <div class="tile"><p class="label">{T('Unseen phrasing', '학습에 없던 표현')}</p><div class="ba"><s>34.3</s><b>94.2%</b></div><span>{T('288 test scenes, 3,744 questions', 'Test scene 288장, 3,744문항')}</span></div>
  <div class="tile navy"><p class="label">{T('In closed loop', '폐루프에서')}</p><b>242 / 243</b><span>{T('planner calls that matched the correct next skill', '정답 다음 skill과 일치한 planner 호출')}</span></div>
</div>'''

    stage = bars(T('Task success rate by stage', '단계별 Task success rate'), 'QLoRA VLM planner, 2 seeds',
                 [[('b', 26.2)], [('s', 41.0)], [('b', 45.5)], [('s', 68.6)], [('b', 71.8)], [('s', 83.4)]],
                 [T('SmolVLA only', 'SmolVLA 단독'), 'VLM + SmolVLA', 'Control 1', '+ DAgger', 'Control 2', '+ Residual RL'],
                 'Task success rate by stage: SmolVLA only 26.2, VLM plus SmolVLA 41.0, control 1 45.5, plus DAgger 68.6, control 2 71.8, plus Residual RL 83.4.', 'wide')
    s3 = f'''<div class="vla-results">{stage}
<div class="side">
  <div class="pairbox"><h3>{T('Same budget, different data', '같은 학습량, 다른 데이터')}</h3><p>{T('percentage points, sign test on episodes paired by instruction', '단위 %p, 같은 지시문 episode끼리 Sign test')}</p>
    <div class="prow"><span>+ DAgger <i>vs Control 1</i><small>p &lt; 0.001</small></span><span class="ba"><b style="font-size:28px">+23.1</b></span></div>
    <div class="prow"><span>+ Residual RL <i>vs Control 2</i><small>p = 0.03</small></span><span class="ba"><b style="font-size:28px">+11.6</b></span></div></div>
  <div class="pairbox"><h3>{T('SmolVLA only vs final model', 'SmolVLA 단독 대 최종 모델')}</h3><p>{T('by task type', 'Task 종류별')}</p>
    <div class="prow"><span>{T('One can (8 types)', '캔 하나(8종)')}</span><span class="ba"><s>33.7</s><b style="font-size:26px">87.5</b></span></div>
    <div class="prow"><span>{T('Two cans (5 types)', '캔 둘(5종)')}</span><span class="ba"><s>14.5</s><b style="font-size:26px">75.5</b></span></div>
    <p class="foot-note">{T('Final model: VLM planner + SmolVLA distilled with Residual RL (157 episodes)', '최종 모델 = VLM planner + Residual RL로 Distillation한 SmolVLA (157 episodes)')}</p></div>
</div></div>
<dl class="defs">
  <dt>{T('SmolVLA only', 'SmolVLA 단독')}</dt><dd>{T('No VLM planner. One SmolVLA performs the whole task from the full instruction, with the same demos and settings (50k steps). 160 episodes.', 'VLM planner 없이 SmolVLA 하나가 전체 지시문으로 Task 전체를 수행. 같은 Demo, 같은 설정 50k steps. 160 episodes')}</dd>
  <dt>Control 1</dt><dd>{T('The 50k VLA trained 8k more steps on the original demos only. Same budget as DAgger, without takeover data.', '50k VLA를 기존 Demo만으로 8k steps 더 학습. DAgger와 학습량은 같고 Takeover 데이터만 없음')}</dd>
  <dt>Control 2</dt><dd>{T('The DAgger model trained 8k more steps on demos and DAgger data. Same budget, without the Residual RL successes.', 'DAgger 최종 모델을 Demo와 DAgger 데이터로 8k steps 더 학습. Residual RL 성공 Trajectory만 없음')}</dd>
</dl>
<p class="cap-text">{T('QLoRA VLM planner, 2 seeds, about 156 episodes in total.', 'QLoRA VLM planner, seed 2개, 약 156 episodes')}</p>'''

    def fix(n, t1, t2, imgs, lis, before, after, ctrl):
        fm = ''.join(f'<figure><img src="{im}{f}.webp" alt="{lab}" loading="lazy">{tags((lab, k),)}</figure>' for f, lab, k in imgs)
        li = ''.join(f'<li>{T(a, b)}</li>' for a, b in lis)
        return f'<div class="fix"><div class="fix-media">{fm}</div><div class="fix-body"><p class="label">Step {n}</p><h3>{T(t1, t2)}</h3><ul>{li}</ul><div class="ba"><s>{before}</s><b>{after}%</b><span>{T("control", "Control")} {ctrl}%</span></div></div></div>'

    s4 = f'''<p>{T('Fine-tuning the planner did not move task success past 41.0%. Both fixes feed the VLA new data; neither changes the VLA with RL directly.', 'Planner Fine-tuning만으로는 Task success rate가 41.0%에서 움직이지 않았음. 두 단계 모두 VLA에 새 데이터를 주는 방식이고, RL로 VLA를 직접 바꾸지 않음')}</p>
<div class="block fixes">
{fix(1, 'Takeover DAgger', 'Takeover DAgger', [('v_takeover_0_25', 'control: VLA', 'red'), ('v_takeover_0_75', 'control: teacher', 'sim')],
     [('The VLA approaches the can for 0.7 to 2.1 s, then a scripted teacher re-plans from that state and finishes', 'VLA가 0.7~2.1초 캔에 접근한 뒤, Scripted teacher가 그 상태에서 다시 계획해 마무리'),
      ('The teacher succeeded in 1,200 of 1,220; only the teacher part is saved, so the VLA learns to recover from its own mistakes', 'Teacher 성공 1,200 / 1,220, Teacher 구간만 저장해 VLA가 자기 실수에서 회복하는 법을 학습'),
      ('Retrained from the 50k VLA on demos and corrections, 8k steps', '50k VLA에서 Demo와 교정 데이터로 8k steps 재학습')], '41.0', '68.6', '45.5')}
{fix(2, 'Residual RL, then distillation', 'Residual RL 뒤 Distillation', [('v_res_zero_env0_0_75', 'VLA alone', ''), ('v_res_rl_env0_0_75', 'VLA + residual', 'red')],
     [('SmolVLA frozen; a small residual policy trained with PPO adds up to 0.06 rad on the 6 active arm joints and sees the privileged can state', 'SmolVLA는 고정, PPO로 학습한 작은 Residual policy가 활성 팔 관절 6개에 최대 0.06 rad를 더함(캔 상태는 Privileged)'),
      ('One skill, 400 trials: 50.1% to 89.3% (24 envs, 417 updates)', 'skill 하나 400회: 50.1%에서 89.3%로(환경 24개, 417 updates)'),
      ('1,404 successful trajectories, labeled with what was executed, distilled into the VLA, which keeps image and joint state as its only inputs', '성공 Trajectory 1,404개(실행된 명령을 라벨로)를 VLA에 Distillation, VLA 입력은 영상과 관절 상태 그대로')], '68.6', '83.4', '71.8')}
</div>'''

    s5 = case_steps([
        ('After fine-tuning the VLM planner, task success stayed at 41.0%.', 'VLM planner를 Fine-tuning한 뒤에도 Task success rate가 41.0%에 머묾'),
        ('An oracle planner gave the same 42.9%, and planner decisions were right in 242 of 243 calls. The bottleneck was grasping: cans were knocked over.', 'Oracle planner를 줘도 42.9%로 같고, Planner 결정도 243번 중 242번 정답. 병목은 Grasp(캔 넘어뜨림)'),
        ('Takeover DAgger with a scripted teacher, then distillation of Residual RL successes on top of the frozen VLA.', 'Scripted teacher가 넘겨받는 Takeover DAgger, 이어서 Frozen VLA 위 Residual RL의 성공 Trajectory를 VLA에 Distillation'),
        ('41.0% to 68.6% to 83.4%, above the same-budget controls (45.5%, 71.8%).', '41.0%에서 68.6%, 83.4%로. 같은 학습량 Control(45.5%, 71.8%)보다 높음')])

    s6 = f'''<div class="callout"><p class="label">{T('Scope', '범위')}</p><p>{T('All numbers come from Isaac Lab simulation.', '모든 수치는 Isaac Lab Simulation 결과')}<small>{T('Fine-tuned VLM planner, about 156 episodes over 2 seeds. RL never changes the VLA weights. The privileged can state is used only by the simulation teacher.', 'Fine-tuning한 VLM planner, seed 2개 약 156 episodes. RL은 VLA weight를 바꾸지 않음. Privileged 캔 상태는 Simulation teacher만 사용')}</small></p></div>
<div class="block figs2">
  {zoomfig(im + 'vla_system.webp', 2400, 1014, 'VLA system figure', T('System, planner examples and one closed-loop episode', '계층 구조, Planner 예시, 폐루프 한 에피소드'))}
  {zoomfig(im + 'vla_fixes.webp', 2400, 1014, 'Grasp fixes figure', T('The two data fixes in detail', '두 교정 단계 상세'))}
</div>'''

    sections = [sec('system', '01', 'System', '구조', s1), sec('planner', '02', 'Planner', 'Planner', s2), sec('results', '03', 'Results', '결과', s3),
                sec('fixes', '04', 'Fixing grasp failures with data', 'Grasp 실패를 데이터로 고친 두 단계', s4),
                sec('debug', '05', 'Problem solving: finding the bottleneck', '문제 해결: 병목 찾기', s5), sec('scope', '06', 'Scope and figures', '범위와 상세 그림', s6)]
    toc_items = [('system', '01 System', '01 구조'), ('planner', '02 Planner', '02 Planner'), ('results', '03 Results', '03 결과'),
                 ('fixes', '04 Fixes', '04 교정'), ('debug', '05 Problem solving', '05 문제 해결'), ('scope', '06 Scope', '06 범위')]
    nxt = next_link('ur2026.html', '', '', 'Upper-Humanoid Real2Sim2Real', 'UR 2026, co-author: perception, sweeping assets, sweeping task', 'UR 2026 공동저자: Perception, Sweeping asset, Sweeping task')
    case_page(p, 'vla.html', 'VLM planner + SmolVLA | Sunghyun Park', 'Language-instructed bimanual manipulation in Isaac Lab: Qwen3-VL-4B planner and SmolVLA executor, task success from 41.0% to 83.4% with DAgger and Residual RL distillation.', header, hero, sections, toc_items, nxt)


# ================================================================ UR 2026

def build_ur():
    p = '../'
    im = '../assets/img/'
    header = cs_header(
        f'{T("Co-author", "공동저자")}<span class="sep">/</span>UR 2026 Work-in-Progress<span class="sep">/</span>{T("Osaka, July 2026", "Osaka, 2026.07")}',
        'Upper-Humanoid Real2Sim2Real', 'An Upper-Humanoid Real2Sim2Real System for Bimanual Long-Horizon Manipulation',
        T('A team system that goes from real demonstrations to a policy on the real robot: a digital twin with real-time object tracking, demonstration augmentation and policy training in simulation, and direct transfer to hardware. I worked on three parts of it.',
          '실물 Demonstration에서 실제 로봇 Policy까지 이어지는 팀 시스템: 실시간 물체 추적을 붙인 Digital twin, Simulation에서 Demo 증강과 Policy 학습, 하드웨어로 바로 전이. 그중 세 부분을 맡음'))
    header += meta_grid([
        (('Role', '역할'), ('Co-author, 4th of 8', '공동저자(8명 중 4저자)')),
        (('My parts', '담당'), ('Perception, sweeping assets, sweeping task', 'Perception, Sweeping asset, Sweeping task')),
        (('Robot', '로봇'), ('UR3 × 2, Allegro Hand × 2', 'UR3 × 2, Allegro Hand × 2')),
        (('Tools', '도구'), ('YOLO, FoundationPose, RealSense D415, Isaac Lab, USD, Fusion', 'YOLO, FoundationPose, RealSense D415, Isaac Lab, USD, Fusion')),
    ])
    hero = f'''<div class="figframe"><img src="{im}u_sweep_strip.webp" alt="Bimanual sweeping in simulation (top) and on the real robot (bottom)" width="2400" height="613"></div>
<p class="cap-text">{T('Bimanual Sweeping in simulation (top) and on the real robot (bottom). Figure from the UR 2026 paper.', 'Bimanual Sweeping, Simulation(위)과 실물(아래). UR 2026 논문 그림')}</p>'''

    s1 = f'''<div class="sys three">
  <div class="snode">{image(im + 'u_twin.webp', 'Real setup and its digital twin', tags(('My part 01', 'red'),), 'r1610')}<div class="snode-body"><p class="label">1 Real2Sim</p><h3>{T('Digital twin and object tracking', 'Digital twin과 물체 추적')}</h3><p>{T('A kinematic digital twin with real-time 6D object tracking, so simulation matches the physical setup.', '실시간 6D 물체 추적을 붙인 Kinematic digital twin으로 Simulation을 실제 구성에 맞춤')}</p></div></div>
  <div class="snode">{image(im + 'u_col_scene.webp', 'Collision shapes in Isaac Lab', tags(('My part 02, 03', 'red'),), 'r1610')}<div class="snode-body"><p class="label">2 Sim training</p><h3>{T('Demo augmentation and policy training', 'Demo 증강과 Policy 학습')}</h3><p>{T('Human seed demonstrations stitched by subtask, then a TC-TRL policy trained with RL.', '사람 시연을 subtask 단위로 이어 늘리고, TC-TRL Policy를 RL로 학습')}</p></div></div>
  <div class="snode">{image(im + 'u_sweep_real.webp', 'Sweeping on the real robot', tags(('Real robot', 'real'),), 'r1610')}<div class="snode-body"><p class="label">3 Sim2Real</p><h3>{T('Policy on the real robot', '실제 로봇 Policy')}</h3><p>{T('The learned policy transferred directly to hardware.', '학습한 Policy를 하드웨어로 바로 전이')}</p></div></div>
</div>'''

    s2 = f'''<div class="compare">
  <div class="ccard"><div class="media r1610" style="background:#111"><img src="{im}u_tracking.webp" alt="Real-time 6D tracking with bounding boxes" loading="lazy" style="object-fit:contain">{tags(('01 Perception', 'red'),)}</div><div class="ccard-body"><p class="label">01</p><h3>Perception</h3><p class="prose">{T('FoundationPose needs a target region marked on the first frame before it can track. I trained a YOLO detector for the target objects so 6D tracking starts from its detections, with no one in the loop. The team used this as policy input in the real-robot experiments.', 'FoundationPose는 첫 프레임에 대상 영역을 지정해야 추적을 시작. 대상 물체 검출용 YOLO를 직접 학습해 검출 결과로 추적이 자동 시작되게 함. 팀의 실제 로봇 실험에서 Policy 입력으로 사용')}</p></div></div>
  <div class="ccard"><div class="media r1610" style="background:#000"><img src="{im}a_col_broom.webp" alt="Broom collider" loading="lazy" style="object-fit:contain">{tags(('02 Sweeping assets', 'red'),)}</div><div class="ccard-body"><p class="label">02</p><h3>{T('Sweeping assets', 'Sweeping asset')}</h3><p class="prose">{T('Broom modeled in CAD and set up as a USD asset, with the bristles approximated as one rigid body. Details on the simulation assets page.', '빗자루를 CAD로 모델링해 USD asset으로 구성, 솔은 하나의 Rigid body로 근사. 자세한 내용은 Simulation asset 페이지')}</p></div></div>
</div>
<div class="block ccard" style="display:grid;grid-template-columns:1.4fr 1fr">{image(im + 'u_col_render.webp', 'Sweeping scene in Isaac Lab', tags(('03 Sweeping task', 'red'),), 'r169')}<div class="ccard-body"><p class="label">03</p><h3>{T('Sweeping task', 'Sweeping task')}</h3><p class="prose">{T('A bimanual sweeping task with stage-wise rewards and penalties, and policy training on it.', '양팔 쓸기 작업, 단계별 Reward와 Penalty 설계, Policy 학습')}</p></div></div>'''

    team = bars(T('Team result, real robot', '팀 결과, 실물'), 'success rate (%)', [[('b', 20.0), ('s', 85.7)], [('b', 37.1), ('s', 82.8)]], ['Bimanual Sweeping', 'Cup Handover'],
                'Real robot success: Bimanual Sweeping, BC only 20.0, system 85.7. Cup Handover, BC only 37.1, system 82.8.')
    s3 = f'''<div class="charts" style="grid-template-columns:1.2fr 1fr">{team}
<div class="side"><div class="tile"><p class="label">Bimanual Sweeping</p><b>85.7%</b><span>{T('vs 20.0% with BC only', 'BC only 20.0%')}</span></div><div class="tile"><p class="label">Cup Handover</p><b>82.8%</b><span>{T('vs 37.1% with BC only', 'BC only 37.1%')}</span></div></div></div>
<ul class="legend"><li><i class="sw b"></i>BC only</li><li><i class="sw s"></i>{T('Real2Sim2Real system', 'Real2Sim2Real 시스템')}</li></ul>'''

    s4 = case_steps([
        ('FoundationPose starts tracking only after someone marks the target region on the first frame.', 'FoundationPose는 첫 프레임에 대상 영역을 지정해야 추적을 시작'),
        ('For the policy to get object poses at every step, tracking had to start without a person.', 'Policy가 매 순간 물체 자세를 받으려면 사람 개입 없이 추적이 시작돼야 함'),
        ('Trained a YOLO detector for the target objects and started tracking from its detections.', '대상 물체 검출용 YOLO를 직접 학습해 검출 결과로 추적 시작'),
        ('Tracking starts as soon as the object is in view, and the team used it as policy input on the real robot.', '물체가 카메라에 잡히면 자동 추적, 팀의 실제 로봇 실험에서 Policy 입력으로 사용')])

    s5 = f'''<div class="figs2">
  {zoomfig(im + 'ur2026.webp', 2400, 831, 'Real2Sim2Real system with my parts marked', T('The system with my three parts marked', '시스템과 내 담당 세 부분'))}
  {zoomfig(im + 'u_tasks.webp', 2400, 1174, 'UR 2026 tasks in simulation and on the real robot', T('Cup Handover and Bimanual Sweeping, simulation and real. Figure from the UR 2026 paper.', 'Cup Handover와 Bimanual Sweeping, Simulation과 실물. UR 2026 논문 그림'))}
</div>'''
    sections = [sec('system', '01', 'System', '시스템', s1), sec('parts', '02', 'My three parts', '내 담당 세 부분', s2), sec('results', '03', 'Results', '결과', s3),
                sec('debug', '04', 'Problem solving: starting tracking on its own', '문제 해결: 추적 자동 시작', s4), sec('figures', '05', 'Figures', '그림', s5)]
    toc_items = [('system', '01 System', '01 시스템'), ('parts', '02 My parts', '02 내 담당'), ('results', '03 Results', '03 결과'), ('debug', '04 Problem solving', '04 문제 해결'), ('figures', '05 Figures', '05 그림')]
    nxt = next_link('sim-assets.html', '', '', T('Real tools, rebuilt for Isaac Lab', '실물 도구를 Isaac Lab으로'), 'Broom, dustpan, broom supporter, cans and photogrammetry', '빗자루, 쓰레받기, 서포터, 캔, Photogrammetry')
    case_page(p, 'ur2026.html', 'UR 2026 Real2Sim2Real | Sunghyun Park', 'UR 2026 Work-in-Progress, co-author: YOLO-started FoundationPose tracking, sweeping assets and the bimanual sweeping task.', header, hero, sections, toc_items, nxt)


# ================================================================ SIM ASSETS

def build_assets():
    p = '../'
    v, im = '../assets/video/', '../assets/img/'
    header = cs_header(
        f'Simulation assets<span class="sep">/</span>2025.09 ~ 2026.10',
        T('Real tools, rebuilt for Isaac Lab', '실물 도구를 Isaac Lab으로'), T('Broom, dustpan, broom supporter and cans', '빗자루, 쓰레받기, 서포터, 캔'),
        T('Objects in simulation have to match the real ones for a policy to transfer. For the sweeping task (UR 2026) and Pick-and-Place (TMAD), appearance follows photos of the real objects, and collision uses separate colliders.',
          'Policy가 실물로 넘어가려면 Simulation의 물체가 실물과 맞아야 함. Sweeping(UR 2026)과 Pick-and-Place(TMAD)에서 겉모습은 실물 사진 기준, 충돌은 별도 Collider로 구성'))
    header += meta_grid([
        (('Role', '역할'), ('CAD modeling, mesh editing, USD and collider setup', 'CAD 모델링, Mesh 편집, USD와 Collider 구성')),
        (('Used in', '사용처'), ('UR 2026 Sweeping, TMAD Pick-and-Place', 'UR 2026 Sweeping, TMAD Pick-and-Place')),
        (('Tools', '도구'), ('Fusion, Blender, MeshLab, Meshroom, USD, Isaac Lab', 'Fusion, Blender, MeshLab, Meshroom, USD, Isaac Lab')),
        (('Assets', '자산'), ('Broom, broom supporter (own CAD), dustpan, cans', '빗자루, 서포터(직접 CAD), 쓰레받기, 캔')),
    ])
    hero = f'''<div class="cs-dark"><div class="cs-hero-grid">
  {image(im + 'a_real_sweep.webp', 'Real sweeping setup', tags(('Real', 'real'),), 'r169')}
  {image(im + 'a_sim_sweep.webp', 'The same setup in Isaac Lab', tags(('Isaac Lab', 'sim'),), 'r169')}
</div></div>
<p class="cap-text">{T('The sweeping setup on the real robot (left) and in Isaac Lab (right).', '실물(왼쪽)과 Isaac Lab(오른쪽)의 쓸기 장면')}</p>'''

    assets = [('a_pair_broom', ('Broom', '빗자루'), 'own CAD', ('Modeled in Fusion (2025.11). Three convex hulls; the bristles are approximated as a rigid body. Bristle-table and bristle-dustpan collisions are excluded.', 'Fusion으로 직접 CAD 모델링(2025.11). Convex hull 3개, 솔은 Rigid body로 근사. 솔과 책상, 쓰레받기 충돌 제외')),
              ('a_pair_dustpan', ('Dustpan', '쓰레받기'), 'public mesh', ('Public mesh edited in Blender and set up as USD, resized to the lab dustpan (21 cm wide), with a convex decomposition collider.', '공개 Mesh를 Blender에서 편집해 USD로 구성. 실험실 쓰레받기 크기(폭 21 cm)로 다시 맞추고 Convex decomposition Collider 적용')),
              ('a_pair_support', ('Broom supporter', '빗자루 서포터'), 'own CAD', ('L-shaped stand modeled in Fusion (2025.12). Fixed as a kinematic body to hold the broom upright at the start.', 'Fusion으로 L자 거치대 모델링(2025.12). Kinematic body로 고정해 빗자루를 세워 두는 시작 자세에 사용')),
              ('a_pair_can', ('Can', '캔'), 'public mesh + label', ('Real label textures on the UV of a public mesh, in red and green, scaled per axis to the real can.', '공개 Mesh의 UV에 실물 라벨 텍스처를 빨강, 초록 두 벌로 입힘. 실물 캔의 지름과 높이에 맞춰 축별 Scale 조정'))]
    cards = ''.join(f'<div class="asset"><img src="{im}{f}.webp" alt="{t[0]}: real and simulation" loading="lazy"><div class="asset-body"><h3>{T(*t)} <span class="chip">{tag}</span></h3><p>{T(*d)}</p></div></div>' for f, t, tag, d in assets)
    s1 = f'<div class="asset-grid">{cards}</div>'

    s2 = f'''<p>{T('The mesh the camera renders comes from the real object. The shape the physics engine uses for contact is a separate, simpler collider.', '카메라가 렌더링하는 Mesh는 실물 기준, 물리 엔진이 접촉에 쓰는 형상은 별도의 단순한 Collider')}</p>
<div class="block compare">
  <figure class="ccard">{image(im + 'a_overlay_dustpan.webp', 'Dustpan with its collider overlaid', tags(('Collider overlay', ''),), 'r169')}<div class="ccard-body"><h3>{T('Dustpan', '쓰레받기')}</h3><p class="prose">{T('Convex decomposition collider (green) over the visual mesh.', 'Visual mesh 위의 Convex decomposition Collider(초록)')}</p></div></figure>
  <figure class="ccard"><div class="media r169" style="background:#000"><img src="{im}a_col_broom.webp" alt="Broom collider" loading="lazy" style="object-fit:contain">{tags(('Collider', ''),)}</div><div class="ccard-body"><h3>{T('Broom', '빗자루')}</h3><p class="prose">{T('Three convex hulls; the bristles are one rigid body.', 'Convex hull 3개, 솔은 하나의 Rigid body')}</p></div></figure>
</div>'''

    s3 = f'''<div class="vgrid" style="grid-template-columns:1fr">{video(v + 'sim_sweep.mp4', v + 'sim_sweep.jpg', 'Isaac Lab sweeping scene', tags(('Isaac Lab', 'sim'), ('Scripted demo', '')), '', 'r169', 'metadata')}</div>
<p class="cap-text">{T('The sweeping scene in Isaac Lab with the lab-size dustpan, run by a scripted demonstration.', '실험실 크기 쓰레받기를 넣은 Isaac Lab 쓸기 장면, Scripted demonstration으로 실행')}</p>'''

    s4 = f'''<p>{T('For another lab project, I photographed real objects, reconstructed them with Meshroom, cleaned them up in MeshLab and Blender, and converted them to OBJ and USD for pose estimation and simulation.', '연구실의 다른 과제에서 자세 추정과 Simulation에 쓸 실물 물체 Mesh가 필요해 직접 촬영 후 Meshroom으로 복원하고, MeshLab과 Blender로 정리해 OBJ와 USD로 변환')}</p>
<div class="chain"><span>photos</span><i></i><span>Meshroom SfM</span><i></i><span>MeshLab cleanup</span><i></i><span>Blender, OBJ</span><i></i><span>USD</span></div>
<div class="block trio">
  <figure><img src="{im}a_tiger_photo.webp" alt="Photo of the object" loading="lazy"><figcaption>photo</figcaption></figure>
  <figure><img src="{im}a_tiger_mesh.webp" alt="Raw reconstructed mesh" loading="lazy"><figcaption>raw mesh</figcaption></figure>
  <figure><img src="{im}a_tiger_tex.webp" alt="Textured model" loading="lazy"><figcaption>textured model</figcaption></figure>
</div>
<div class="block"><div class="figframe"><img src="{im}a_meshroom.webp" alt="Meshroom reconstruction from 57 photos" loading="lazy"></div><p class="cap-text">{T('Meshroom, 57 photos of a sauce bottle. Lab project, 2025.09 to 10.', 'Meshroom, 소스 병 사진 57장. 연구실 과제, 2025.09 ~ 10')}</p></div>'''

    sections = [sec('assets', '01', 'Four assets', '네 가지 자산', s1), sec('collision', '02', 'Visual mesh and collider', 'Visual mesh와 Collider', s2),
                sec('motion', '03', 'In the simulator', 'Simulator 안에서', s3), sec('photogrammetry', '04', 'Photogrammetry 3D reconstruction', 'Photogrammetry 3D 복원', s4)]
    toc_items = [('assets', '01 Assets', '01 자산'), ('collision', '02 Colliders', '02 Collider'), ('motion', '03 In the simulator', '03 Simulator'), ('photogrammetry', '04 Photogrammetry', '04 Photogrammetry')]
    nxt = next_link('tmad.html', '', '', 'TMAD', 'Back to the main research: zero-shot sim-to-real for two arms', '주 연구로: 양팔 Zero-shot Sim-to-Real')
    case_page(p, 'sim-assets.html', 'Simulation assets | Sunghyun Park', 'Real tools rebuilt for Isaac Lab: CAD broom and supporter, dustpan resized to the lab one, cans with real labels, colliders, photogrammetry.', header, hero, sections, toc_items, nxt)


if __name__ == '__main__':
    os.makedirs(os.path.join(ROOT, 'work'), exist_ok=True)
    build_home()
    build_tmad()
    build_vla()
    build_ur()
    build_assets()
    print('built')
