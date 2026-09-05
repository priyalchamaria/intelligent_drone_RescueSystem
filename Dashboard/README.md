# Mission Dashboard

A browser view of a live Unity run: fleet status, a 2D situation map, the
priority queue, the alerts feed, why the last dispatch went to the drone it went
to, and the eight Phase 7 metrics once a run finishes.

## Running it

Two things have to be going at once.

1. **Unity.** Press play on `Assets/Scenes/DisasterResponse.unity`. The
   `DashboardStateWriter` on the `MissionSystem` object republishes
   `Dashboard/data/state.json` about three times a second. It writes the file and
   nothing else; the simulation neither knows nor cares whether anything reads it.

2. **A local web server, from this folder.**

   ```
   cd Dashboard
   python -m http.server 8000
   ```

   Then open <http://localhost:8000>.

   `serve.bat` does the same thing on a double-click.

### Why a server, rather than opening the file

Browsers refuse `fetch` against `file://` for security reasons, so opening
`index.html` directly gives a page that loads and then never shows any data. Any
static server will do; `python -m http.server` is used above only because it
needs nothing installed.

## Reading it

**Live** is the running mission.

- **Situation map.** Triangles are drones, coloured as their route lines are in
  Unity, so D-03 is the same colour in both. Circles are casualties, coloured by
  triage priority and greyed once delivered; a ring around one means it is still
  waiting. A dashed line joins a drone to the casualty it is assigned. Squares are
  the hospital and the charging pads. Grey blocks are rubble, orange ones are fire
  that a FireSpread event has added.
- **Fleet status.** One row per drone, straight from the shared `DroneList` the
  Mission Planner reads. Range is the Part 1 figure, `(battery / 100) × MAX_RANGE`.
- **Priority queue.** In served order, not heap order.
- **Alerts and progress.** The four Part 1 dynamic events are marked and
  coloured; ordinary progress is not.
- **Assignment explainability.** Every drone measured for the most recent
  dispatch. Excluded drones show which hard filter caught them. Scored drones show
  the three factors, a bar splitting the score into its distance, battery and risk
  contributions, and the total. Lowest wins, and the winner is highlighted.

**Analytics** appears when a run finishes: the eight metrics, response time per
casualty, and the battery identity `start + recharged − spent = remaining`.

## Dispatch modes

The `MissionPlanner` component carries a `Dispatch Mode` toggle, read on every
dispatch and set before pressing play.

| Mode | Rule |
| --- | --- |
| `Scored` | Part 1 Step 3. Lowest `W1 x distance + W2 x batteryUtilisation + W3 x risk` wins. |
| `NearestIdle` | The Phase 9 baseline. Closest idle drone, nothing else considered. |

Both modes see the same Part 1 Step 2 hard filter, so a comparison run measures
the selection rule and nothing else. The three score factors are computed in both
modes, so a baseline run still records what the planner would have chosen: the
explainability panel marks that drone and says so, and the header names the mode
in force. The summary CSV records it per run, taken from the planner rather than
typed anywhere, so the two cannot disagree.

## What it does not do

It does not talk back. There is no control here, no way to dispatch a drone or
fire an event from the browser, and that is deliberate: a measurement tool that
can change the run is not measuring it any more. Events are triggered in Unity,
by the scripted timeline or the number keys.

## Files

| File | What it is |
| --- | --- |
| `index.html` | Page structure. |
| `style.css` | Styling, using the simulation's own palette. |
| `app.js` | Polling and rendering. No dependencies, no build step. |
| `data/state.json` | Written by Unity. Not committed; every run replaces it. |
