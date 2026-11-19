#!/usr/bin/env node
// Assemble the paste-ready files in agent/solutions from agent/src.
// A source file may pull in a shared library with a line "//#include lib/x.js"
// (the game needs one self-contained script per level), and one source file
// may serve several levels.
'use strict';
const fs = require('fs'), path = require('path');
const SRC = path.join(__dirname, '..', 'src'), OUT = path.join(__dirname, '..', 'solutions');
const LEVELS = {
  'TutorialBlock.js': ['TutorialBlockWithFriction', 'TutorialBlockWithoutFriction', 'TutorialBlockOnSlope'],
  'CruiseControl.js': ['CruiseControlIntro', 'CruiseControl2'],
  'SinglePendulum.js': ['StabilizeSinglePendulum', 'SwingUpSinglePendulum'],
  'DoublePendulum.js': ['StabilizeDoublePendulum', 'SwingUpDoublePendulum'],
  'RocketLanding.js': ['RocketLandingNormal', 'RocketLandingUpsideDown', 'RocketLandingMulti', 'RocketLandingHoverslam'],
};
fs.mkdirSync(OUT, { recursive: true });
for (const f of fs.readdirSync(SRC)) {
  if (!f.endsWith('.js')) continue;
  const code = fs.readFileSync(path.join(SRC, f), 'utf8').replace(/^\/\/#include (.*)$/mg, (_, inc) => fs.readFileSync(path.join(SRC, inc.trim()), 'utf8').trimEnd());
  for (const level of LEVELS[f] || [f.replace('.js', '')]) fs.writeFileSync(path.join(OUT, level + '.js'), code);
}
