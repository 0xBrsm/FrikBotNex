FrikBotNex Attributions
======================

Original Quake Game Code
------------------------
Copyright (C) 1996-1997 id Software, Inc.
Licensed under the GNU General Public License, version 2.
See src/qc/LICENSE.txt for the full license text.

The base game code in src/qc/ is from the CleanFixedQuakeC project
by Jason Brownlee (Jason2Brownlee), a cleaned-up release of the
original GPL source code published by id Software. Includes fixes
inspired by "Quakec V1 01 Clean" by gnounc and the QW 2.30 GPL release.

FrikBot X (v0.10.2)
--------------------
By Ryan "FrikaC" Smith
Released into the public domain.

FrikBot is a QuakeC bot framework providing waypoint navigation,
combat AI, physics prediction, and an in-game editor. The original
FrikBot code forms the foundation of the bot system in src/frikbot/.

Special thanks (from bot_phys.qc): Asdf, Frog, and Alan "Strider" Kivlin
for contributions to the client movement physics emulation.

FrikBot X+
----------
By "Igor9"

Updated version of FrikBot X with AI fixes and improvements,
including weapon range corrections and item priority tuning.
Changes are marked with "//~" comments in the source (e.g.
"ai fix start/end", "hip_fbx bugfix").

FrikBot X++
-----------
By Joel Baxter ("Johnny Law")
Released January 2013.

Restored automatic map cycling that was inadvertently removed in
FrikBot X+. FrikBot X++ is the direct ancestor of this codebase.

FrikBotNex
----------
By Brian St. Marie (0xBrsm) and Claude (Anthropic)

Combat AI enhancements, human emulation systems, opponent modeling,
team coordination, personality system, and utility-scored state machine.
Released into the public domain.
