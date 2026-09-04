/*
   Mission dashboard.

   Polls data/state.json, which Unity republishes a few times a second, and
   redraws from whatever it finds. There is no state kept between polls beyond
   the last document received: every panel is a pure function of one snapshot.
   That is what makes the page safe to open late, reload mid-run, or leave open
   across several runs, none of which the simulation is told about.

   No build step and no dependencies. It is served as three files so that
   starting it during a demo is one command, and so the page cannot fail because
   something did not download.
*/

const SOURCE = 'data/state.json';
const POLL_MS = 300;

/* The simulation's own palette. Same colours, so the map and the Unity view
   read as one thing rather than two. */
const ROUTE = ['#26f2ff', '#ff51d9', '#ffeb38', '#51ff73', '#a3a3ff', '#ff7a73'];
const PRIORITY = { Critical: '#db2424', Serious: '#f2a31a', Stable: '#3dbd52' };
const RESCUED = '#9ea89e';
const HOSPITAL = '#ebf2ff';
const CHARGING = '#21b88c';
const FIRE = '#e6541a';
const RUBBLE = '#4a5060';

let last = null;
let misses = 0;

/* A drone keeps its colour for the whole run, taken from the digits in its id,
   exactly as FleetPalette does in Unity. D-03 is the same colour in both. */
function droneColor(id) {
    const m = /(\d+)/.exec(id || '');
    const n = m ? parseInt(m[1], 10) : 0;
    return ROUTE[((n % ROUTE.length) + ROUTE.length) % ROUTE.length];
}

const $ = id => document.getElementById(id);
const num = (v, d = 1) => (v === null || v === undefined || isNaN(v)) ? '—' : Number(v).toFixed(d);
const esc = s => String(s === null || s === undefined ? '' : s)
    .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');

/* ------------------------------------------------------------------ polling */

async function poll() {
    try {
        // The query string defeats the browser cache, which otherwise serves the
        // first snapshot for the rest of the run and makes the page look frozen.
        const res = await fetch(SOURCE + '?t=' + Date.now(), { cache: 'no-store' });
        if (!res.ok) throw new Error(res.status);

        last = await res.json();
        misses = 0;
    } catch (e) {
        misses++;
        if (misses > 3) offline();
        return;
    }

    // Rendering is deliberately outside the fetch guard. A fault in a panel is a
    // fault in this file, and folding it into the catch above would report it as a
    // lost connection and send the reader to look at Unity instead.
    render(last);
}

function offline() {
    const link = $('link');
    link.className = 'pill off';
    $('linkText').textContent = last ? 'Unity stopped' : 'waiting for Unity';
}

/* ------------------------------------------------------------------- render */

function render(s) {
    const link = $('link');
    const complete = s.runComplete && s.runStarted;

    link.className = 'pill ' + (complete ? 'done' : 'live');
    $('linkText').textContent = complete ? 'run complete' : (s.runStarted ? 'live' : 'standing by');

    $('scenario').textContent = s.scenario + '  ·  updated ' + s.generatedAt;
    $('clock').textContent = num(s.simTime) + 's';
    $('statDispatch').textContent = s.dispatches;
    $('statReassign').textContent = s.reassignments;

    const delivered = s.patients.filter(p => p.state === 'Delivered').length;
    $('statDelivered').textContent = delivered + ' / ' + s.patients.length;

    drawMap(s);
    renderFleet(s);
    renderQueue(s);
    renderFeed(s);
    renderExplain(s);
    renderAnalytics(s);
}

/* ---------------------------------------------------------------------- map */

function drawMap(s) {
    const c = $('map');
    const g = c.getContext('2d');
    const w = s.world || {};
    const worldW = w.width || 100;
    const worldH = w.height || 100;
    const ox = w.originX || 0;
    const oz = w.originZ || 0;

    $('mapNote').textContent = Math.round(worldW) + ' × ' + Math.round(worldH) + ' world units';

    const size = c.width;
    const k = size / Math.max(worldW, worldH);

    // World z runs away from the camera; screen y runs down. Flipping it keeps
    // the map the same way up as the Unity view instead of mirrored.
    const X = x => (x - ox) * k;
    const Y = z => size - (z - oz) * k;

    g.fillStyle = '#0d0f14';
    g.fillRect(0, 0, size, size);

    // grid, every 10 units
    g.strokeStyle = 'rgba(255,255,255,0.045)';
    g.lineWidth = 1;
    for (let v = 0; v <= Math.max(worldW, worldH); v += 10) {
        g.beginPath(); g.moveTo(X(ox + v), 0); g.lineTo(X(ox + v), size); g.stroke();
        g.beginPath(); g.moveTo(0, Y(oz + v)); g.lineTo(size, Y(oz + v)); g.stroke();
    }

    (s.world.obstacles || []).forEach(o => {
        g.fillStyle = o.fire ? 'rgba(230,84,26,0.30)' : 'rgba(74,80,96,0.55)';
        g.strokeStyle = o.fire ? FIRE : '#5b6377';
        g.lineWidth = o.fire ? 2 : 1;

        if (o.circle) {
            g.beginPath();
            g.arc(X(o.x), Y(o.z), Math.max(2, o.r * k), 0, Math.PI * 2);
            g.fill(); g.stroke();
        } else {
            const x = X(o.x - o.hx), y = Y(o.z + o.hz);
            g.fillRect(x, y, o.hx * 2 * k, o.hz * 2 * k);
            g.strokeRect(x, y, o.hx * 2 * k, o.hz * 2 * k);
        }
    });

    (s.world.hospitals || []).forEach(p => site(g, X(p.x), Y(p.z), HOSPITAL, 'H'));
    (s.world.chargingStations || []).forEach(p => site(g, X(p.x), Y(p.z), CHARGING, '⚡'));

    // leader lines first, so a marker is never drawn under one
    g.lineWidth = 1.5;
    (s.fleet || []).forEach(d => {
        if (!d.task) return;
        const target = s.patients.find(p => p.id === d.task);
        if (!target) return;

        g.strokeStyle = droneColor(d.id);
        g.globalAlpha = 0.4;
        g.setLineDash([4, 4]);
        g.beginPath();
        g.moveTo(X(d.x), Y(d.z));
        g.lineTo(X(target.x), Y(target.z));
        g.stroke();
        g.setLineDash([]);
        g.globalAlpha = 1;
    });

    (s.patients || []).forEach(p => {
        const done = p.state === 'Delivered';
        const color = done ? RESCUED : (PRIORITY[p.priority] || '#fff');
        const x = X(p.x), y = Y(p.z);

        g.beginPath();
        g.arc(x, y, done ? 4 : 6, 0, Math.PI * 2);
        g.fillStyle = color;
        g.globalAlpha = done ? 0.55 : 1;
        g.fill();
        g.globalAlpha = 1;

        if (!done && p.state === 'Waiting') {
            g.beginPath();
            g.arc(x, y, 10, 0, Math.PI * 2);
            g.strokeStyle = color;
            g.globalAlpha = 0.4;
            g.lineWidth = 1.5;
            g.stroke();
            g.globalAlpha = 1;
        }

        label(g, p.id, x, y - 11, done ? RESCUED : color);
    });

    (s.fleet || []).forEach(d => {
        const x = X(d.x), y = Y(d.z);
        const color = droneColor(d.id);
        const dim = d.status === 'Offline';

        g.globalAlpha = dim ? 0.35 : 1;

        g.beginPath();
        g.moveTo(x, y - 7); g.lineTo(x + 6, y + 5); g.lineTo(x - 6, y + 5);
        g.closePath();
        g.fillStyle = color;
        g.fill();
        g.strokeStyle = '#0d0f14';
        g.lineWidth = 1.5;
        g.stroke();

        if (d.status === 'Charging') {
            g.beginPath();
            g.arc(x, y, 11, 0, Math.PI * 2);
            g.strokeStyle = CHARGING;
            g.lineWidth = 2;
            g.stroke();
        }

        label(g, d.id, x, y + 17, color);
        g.globalAlpha = 1;
    });
}

function site(g, x, y, color, glyph) {
    g.fillStyle = color;
    g.globalAlpha = 0.16;
    g.fillRect(x - 11, y - 11, 22, 22);
    g.globalAlpha = 1;
    g.strokeStyle = color;
    g.lineWidth = 1.5;
    g.strokeRect(x - 11, y - 11, 22, 22);

    g.fillStyle = color;
    g.font = '600 12px Segoe UI, sans-serif';
    g.textAlign = 'center';
    g.textBaseline = 'middle';
    g.fillText(glyph, x, y + 1);
}

function label(g, text, x, y, color) {
    g.font = '600 10px Segoe UI, sans-serif';
    g.textAlign = 'center';
    g.textBaseline = 'middle';

    const w = g.measureText(text).width + 6;
    g.fillStyle = 'rgba(13,15,20,0.72)';
    g.fillRect(x - w / 2, y - 7, w, 13);

    g.fillStyle = color;
    g.fillText(text, x, y);
}

/* -------------------------------------------------------------------- fleet */

function renderFleet(s) {
    const rows = (s.fleet || []).map(d => {
        const color = droneColor(d.id);
        const pct = Math.max(0, Math.min(100, d.battery));
        const fill = pct < 25 ? '#db2424' : (pct < 50 ? '#f2a31a' : CHARGING);

        return '<tr>'
            + '<td><span class="who"><i class="swatch" style="background:' + color + '"></i>' + esc(d.id) + '</span></td>'
            + '<td><span class="chip ' + d.status.toLowerCase() + '">' + esc(d.status) + '</span></td>'
            + '<td class="num"><div class="batt"><div class="fill" style="background:' + fill
            + ';width:' + pct + '%"></div><span>' + num(d.battery, 0) + '%</span></div></td>'
            + '<td class="num">' + num(d.range, 0) + 'u</td>'
            + '<td>' + (d.task ? esc(d.task) + ' <span style="color:var(--ink-faint)">'
                + esc(legName(d.leg)) + '</span>' : '<span style="color:var(--ink-faint)">—</span>') + '</td>'
            + '</tr>';
    }).join('');

    $('fleet').innerHTML = rows || '<tr><td colspan="5" class="empty">No drones registered.</td></tr>';

    const busy = (s.fleet || []).filter(d => d.status === 'Busy').length;
    $('fleetNote').textContent = busy + ' of ' + (s.fleet || []).length + ' flying';
}

function legName(leg) {
    if (leg === 'ToPatient') return 'inbound';
    if (leg === 'ToHospital') return 'to hospital';
    if (leg === 'ToChargingStation') return 'to pad';
    return '';
}

/* -------------------------------------------------------------------- queue */

function renderQueue(s) {
    const q = s.queue || [];

    $('queueNote').textContent = q.length ? q.length + ' waiting' : 'empty';
    $('queue').innerHTML = q.length
        ? q.map((p, i) => '<tr>'
            + '<td style="color:var(--ink-faint)">' + (i + 1) + '</td>'
            + '<td><b>' + esc(p.id) + '</b></td>'
            + '<td><span class="chip ' + p.priority.toLowerCase() + '">' + esc(p.priority) + '</span></td>'
            + '<td class="num">' + num(p.waiting) + 's</td>'
            + '</tr>').join('')
        : '<tr><td colspan="4" class="empty">Queue is empty.</td></tr>';
}

/* --------------------------------------------------------------------- feed */

function renderFeed(s) {
    const feed = s.feed || [];
    const el = $('feed');

    // Only redraw when a line has actually been added. Rewriting the list every
    // poll would fight the user's scrollbar three times a second.
    const signature = feed.length + '|' + (feed.length ? feed[feed.length - 1].text : '');
    if (el.dataset.sig === signature) return;
    el.dataset.sig = signature;

    const stuck = el.scrollTop + el.clientHeight >= el.scrollHeight - 24;

    el.innerHTML = feed.length ? feed.map(f =>
        '<div class="line ' + (f.kind ? 'alert ' + f.kind : '') + '">'
        + '<span class="t">' + num(f.t) + 's</span>'
        + (f.kind ? '<span class="k">' + esc(spaced(f.kind)) + '</span>' : '')
        + '<span class="m">' + esc(f.text) + '</span>'
        + '</div>').join('') : '<div class="empty">No events yet.</div>';

    if (stuck) el.scrollTop = el.scrollHeight;
}

function spaced(kind) {
    return kind.replace(/([a-z])([A-Z])/g, '$1 $2');
}

/* ---------------------------------------------------------- explainability */

function renderExplain(s) {
    const x = s.explain;
    const body = $('explainBody');

    if (!x) {
        $('explainNote').textContent = 'no dispatch yet';
        $('explainFormula').textContent = '';
        body.innerHTML = '<div class="empty">Nothing has been dispatched yet.</div>';
        return;
    }

    $('explainNote').textContent = x.patient + ' at ' + num(x.at) + 's  ·  won by ' + x.winner;
    $('explainFormula').innerHTML =
        'score = <b>' + x.w1 + '</b> × totalDistance + <b>' + x.w2 + '</b> × batteryUtilisation + <b>'
        + x.w3 + '</b> × riskFactor &nbsp;&nbsp;·&nbsp;&nbsp; lowest wins';

    const scored = x.rows.filter(r => r.scored);
    const worst = Math.max(1, ...scored.map(r => r.score));

    const rows = x.rows.map(r => {
        if (!r.scored) {
            return '<tr class="out">'
                + '<td><span class="who"><i class="swatch" style="background:' + droneColor(r.id)
                + ';opacity:.45"></i>' + esc(r.id) + '</span></td>'
                + '<td colspan="5">excluded: ' + esc(verdict(r)) + '</td>'
                + '</tr>';
        }

        const d = (r.distTerm / worst) * 100;
        const b = (r.battTerm / worst) * 100;
        const k = (r.riskTerm / worst) * 100;

        return '<tr class="' + (r.winner ? 'win' : '') + '">'
            + '<td><span class="who"><i class="swatch" style="background:' + droneColor(r.id) + '"></i>'
            + esc(r.id) + (r.winner ? ' ✓' : '') + '</span></td>'
            + '<td class="num">' + num(r.total, 0) + 'u</td>'
            + '<td class="num">' + num(r.battUtil, 2) + '</td>'
            + '<td class="num">' + num(r.risk, 2) + '</td>'
            + '<td><div class="bar" title="distance / battery / risk">'
            + '<i class="d" style="width:' + d + '%"></i>'
            + '<i class="b" style="width:' + b + '%"></i>'
            + '<i class="r" style="width:' + k + '%"></i></div></td>'
            + '<td class="num"><b>' + num(r.score) + '</b></td>'
            + '</tr>';
    }).join('');

    body.innerHTML = '<table><thead><tr>'
        + '<th>Drone</th><th class="num">Total dist</th><th class="num">Batt util</th>'
        + '<th class="num">Risk</th><th>Contribution</th><th class="num">Score</th>'
        + '</tr></thead><tbody>' + rows + '</tbody></table>';
}

function verdict(r) {
    if (r.verdict === 'NotIdle') return 'not idle';
    if (r.verdict === 'OutOfRange')
        return 'needs ' + num(r.total, 0) + 'u, battery gives ' + num(r.range, 0) + 'u';
    return r.verdict;
}

/* ---------------------------------------------------------------- analytics */

function renderAnalytics(s) {
    const a = s.analytics;

    if (!a) {
        $('runNote').textContent = 'no completed run yet';
        $('metrics').innerHTML = '<div class="empty">The eight metrics appear when a run finishes.</div>';
        return;
    }

    $('runNote').textContent = a.runId + '  ·  ' + a.scenario + '  ·  ' + a.dispatchMode + ' dispatch';

    const cards = [
        ['mission completion time', num(a.missionSeconds) + 's', num(a.missionMs, 0) + 'ms wall clock'],
        ['average response time', num(a.avgResponse) + 's', 'worst ' + num(a.worstResponse) + 's over ' + a.responseSamples],
        ['rescue success rate', num(a.successRate, 0) + '%', a.delivered + ' of ' + a.patients + ' delivered'],
        ['battery at mission end', num(a.batteryRemaining, 0) + '%', 'lowest ' + num(a.batteryMin, 0) + '% remaining'],
        ['collisions', String(a.collisions), a.minSeparation >= 0 ? 'closest ' + num(a.minSeparation, 2) + 'u' : 'never two in the air'],
        ['reassignments', String(a.reassignments), 'of ' + a.dispatches + ' dispatches'],
        ['coverage', num(a.coverage, 0) + '%', a.reached + ' of ' + a.patients + ' reached'],
        ['system throughput', num(a.throughput, 2), 'casualties per simulated minute']
    ];

    $('metrics').innerHTML = cards.map(c =>
        '<div class="metric"><span>' + c[0] + '</span><b>' + c[1] + '</b><em>' + c[2] + '</em></div>').join('');

    renderResponses(s);
    renderBattery(a);
}

function renderResponses(s) {
    const rows = (s.patients || []).filter(p => p.response >= 0);
    if (!rows.length) return;

    const worst = Math.max(...rows.map(p => p.response));

    $('responses').innerHTML = '<table><tbody>' + rows.map(p => {
        const color = PRIORITY[p.priority] || RESCUED;
        return '<tr>'
            + '<td class="nowrap" style="width:1%"><b>' + esc(p.id) + '</b></td>'
            + '<td class="nowrap" style="width:1%"><span class="chip ' + p.priority.toLowerCase() + '">'
            + esc(p.priority) + '</span></td>'
            + '<td><div class="rowbar"><div class="track"><i style="width:'
            + (p.response / worst * 100) + '%;background:' + color + '"></i></div></div></td>'
            + '<td class="num nowrap" style="width:1%">' + num(p.response) + 's</td>'
            + '<td class="num nowrap" style="width:1%;color:var(--ink-faint)">' + esc(p.drone || '—') + '</td>'
            + '</tr>';
    }).join('') + '</tbody></table>';
}

function renderBattery(a) {
    // start + recharged - spent = remaining. Shown as the identity rather than as
    // four unrelated numbers, because that is the only form in which they can be
    // checked against each other.
    const parts = [
        ['started', a.batteryStart, '#4c7dff'],
        ['recharged', a.batteryRecharged, CHARGING],
        ['spent', a.batteryUsed, FIRE],
        ['remaining', a.batteryRemaining, HOSPITAL]
    ];

    const worst = Math.max(1, ...parts.map(p => p[1]));

    $('battery').innerHTML = '<table><tbody>' + parts.map(p =>
        '<tr><td style="width:1%">' + p[0] + '</td>'
        + '<td><div class="rowbar"><div class="track"><i style="width:'
        + (p[1] / worst * 100) + '%;background:' + p[2] + '"></i></div></div></td>'
        + '<td class="num" style="width:1%">' + num(p[1]) + '%</td></tr>').join('')
        + '</tbody></table>'
        + '<div class="formula" style="border-top:1px solid var(--line);border-bottom:none">'
        + num(a.batteryStart) + ' started + ' + num(a.batteryRecharged) + ' recharged − '
        + num(a.batteryUsed) + ' spent = <b>' + num(a.batteryRemaining) + ' remaining</b>'
        + '&nbsp;&nbsp;·&nbsp;&nbsp;' + a.recharges + ' recharges</div>';
}

/* --------------------------------------------------------------------- tabs */

document.querySelectorAll('nav button').forEach(b => {
    b.onclick = () => {
        document.querySelectorAll('nav button').forEach(x => x.classList.toggle('on', x === b));
        document.querySelectorAll('.view').forEach(v =>
            v.classList.toggle('on', v.id === 'view-' + b.dataset.view));
    };
});

poll();
setInterval(poll, POLL_MS);
