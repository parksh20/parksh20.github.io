(function () {
  var root = document.documentElement;
  var reduce = window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  var hasIO = 'IntersectionObserver' in window;

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
  buttons.forEach(function (b) { b.addEventListener('click', function () { setLang(b.getAttribute('data-set-lang'), true); }); });

  // nav: light text while it sits over the dark hero
  var nav = document.getElementById('nav');
  var hero = document.querySelector('.hero');
  if (nav && hero) {
    var navCheck = function () {
      var r = hero.getBoundingClientRect();
      nav.classList.toggle('on-dark', r.bottom > nav.offsetHeight);
    };
    window.addEventListener('scroll', navCheck, { passive: true });
    window.addEventListener('resize', navCheck);
    navCheck();
  }

  // videos play only while visible
  var vids = document.querySelectorAll('video.autoplay');
  if (reduce) {
    vids.forEach(function (v) { v.controls = true; });
  } else if (hasIO) {
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
    }, { threshold: 0.2 });
    vids.forEach(function (v) { vio.observe(v); });
  } else {
    vids.forEach(function (v) { v.autoplay = true; if (v.play) v.play(); });
  }

  // charts grow when they come into view
  if (!reduce && hasIO) {
    root.classList.add('js-anim');
    var once = new IntersectionObserver(function (entries) {
      entries.forEach(function (e) {
        if (e.isIntersecting) { e.target.classList.add('in'); once.unobserve(e.target); }
      });
    }, { threshold: 0.15 });
    document.querySelectorAll('.chart').forEach(function (el) { once.observe(el); });
  }

  // table of contents on case study pages
  var toc = document.querySelectorAll('.toc a');
  if (toc.length && hasIO) {
    var map = {};
    toc.forEach(function (a) { map[a.getAttribute('href').slice(1)] = a; });
    var tio = new IntersectionObserver(function (entries) {
      entries.forEach(function (e) {
        if (e.isIntersecting && map[e.target.id]) {
          toc.forEach(function (a) { a.classList.remove('active'); });
          map[e.target.id].classList.add('active');
        }
      });
    }, { rootMargin: '-30% 0px -60% 0px' });
    document.querySelectorAll('.cs-sec[id]').forEach(function (s) { tio.observe(s); });
  }

  // copy email
  var toast = document.getElementById('toast');
  function say(msg) {
    if (!toast) return;
    toast.textContent = msg;
    toast.classList.add('show');
    clearTimeout(say.t);
    say.t = setTimeout(function () { toast.classList.remove('show'); }, 1800);
  }
  document.querySelectorAll('[data-copy]').forEach(function (b) {
    b.addEventListener('click', function () {
      var text = b.getAttribute('data-copy');
      var ko = root.getAttribute('data-lang') === 'ko';
      var done = function () { say(ko ? '주소를 복사했습니다' : 'Copied ' + text); };
      if (navigator.clipboard && navigator.clipboard.writeText) {
        navigator.clipboard.writeText(text).then(done, function () { say(text); });
      } else {
        say(text);
      }
    });
  });

  // figure zoom
  var lb = document.getElementById('lightbox');
  if (lb) {
    var lbImg = lb.querySelector('img');
    var last = null;
    var closeLb = function () {
      lb.hidden = true;
      lbImg.removeAttribute('src');
      document.body.style.overflow = '';
      if (last) last.focus();
    };
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
  }
})();
