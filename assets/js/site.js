(function () {
  var root = document.documentElement;
  var reduce = window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches;

  // language
  var buttons = document.querySelectorAll('[data-set-lang]');
  function setLang(l, save) {
    root.setAttribute('data-lang', l);
    root.lang = l;
    buttons.forEach(function (b) { b.setAttribute('aria-pressed', String(b.getAttribute('data-set-lang') === l)); });
    if (save) {
      try { localStorage.setItem('lang', l); } catch (e) {}
      try {
        var u = new URL(location.href);
        if (u.searchParams.has('lang')) { u.searchParams.set('lang', l); history.replaceState(null, '', u); }
      } catch (e) {}
    }
  }
  setLang(root.getAttribute('data-lang') === 'ko' ? 'ko' : 'en', false);
  buttons.forEach(function (b) {
    b.addEventListener('click', function () { setLang(b.getAttribute('data-set-lang'), true); });
  });

  // nav border and active section
  var nav = document.querySelector('.nav');
  var onScroll = function () { nav.classList.toggle('scrolled', window.scrollY > 8); };
  window.addEventListener('scroll', onScroll, { passive: true });
  onScroll();

  if ('IntersectionObserver' in window) {
    var links = {};
    document.querySelectorAll('.links a').forEach(function (a) { links[a.getAttribute('href').slice(1)] = a; });
    var spy = new IntersectionObserver(function (entries) {
      entries.forEach(function (e) {
        if (e.isIntersecting && links[e.target.id]) {
          Object.keys(links).forEach(function (k) { links[k].classList.remove('active'); });
          links[e.target.id].classList.add('active');
        }
      });
    }, { rootMargin: '-45% 0px -50% 0px' });
    document.querySelectorAll('section[id]').forEach(function (s) { spy.observe(s); });
  }

  // videos: play only while visible
  var vids = document.querySelectorAll('video.autoplay');
  if (reduce) {
    vids.forEach(function (v) { v.controls = true; });
  } else if ('IntersectionObserver' in window) {
    var vio = new IntersectionObserver(function (entries) {
      entries.forEach(function (e) {
        var v = e.target;
        if (e.isIntersecting) {
          if (v.preload === 'none') v.preload = 'auto';
          var p = v.play();
          if (p && p.catch) p.catch(function () { v.controls = true; });
        } else {
          v.pause();
        }
      });
    }, { threshold: 0.25 });
    vids.forEach(function (v) { vio.observe(v); });
  } else {
    vids.forEach(function (v) { v.autoplay = true; v.play && v.play(); });
  }

  // charts grow when they come into view
  if (!reduce && 'IntersectionObserver' in window) {
    root.classList.add('js-anim');
    var cio = new IntersectionObserver(function (entries) {
      entries.forEach(function (e) {
        if (e.isIntersecting) { e.target.classList.add('in'); cio.unobserve(e.target); }
      });
    }, { threshold: 0.35 });
    document.querySelectorAll('.chart').forEach(function (c) { cio.observe(c); });
  }

  // figure lightbox
  var lb = document.getElementById('lightbox');
  var lbImg = lb.querySelector('img');
  var last = null;
  function closeLb() {
    lb.hidden = true;
    lbImg.removeAttribute('src');
    document.body.style.overflow = '';
    if (last) last.focus();
  }
  document.querySelectorAll('[data-zoom]').forEach(function (btn) {
    btn.addEventListener('click', function () {
      last = btn;
      var img = btn.querySelector('img');
      lbImg.src = btn.getAttribute('data-zoom');
      lbImg.alt = img ? img.alt : '';
      lb.hidden = false;
      document.body.style.overflow = 'hidden';
      lb.querySelector('.lb-close').focus();
    });
  });
  lb.addEventListener('click', closeLb);
  document.addEventListener('keydown', function (e) { if (e.key === 'Escape' && !lb.hidden) closeLb(); });
})();
