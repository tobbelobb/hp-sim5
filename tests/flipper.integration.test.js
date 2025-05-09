// flipper.integration.test.js
const puppeteer = require('puppeteer');
const path = require('path');

describe('Flipper Integration Test', () => {
    let browser;
    let page;

    // Increased timeout for Jest, Puppeteer can be slow to start
    jest.setTimeout(120000); // 120 seconds for the entire test suite in this file

    beforeAll(async () => {
        browser = await puppeteer.launch({
            headless: "new", // Use "new" headless mode
            args: [
                '--no-sandbox',
                '--disable-setuid-sandbox'
            ] // Add this for CI environments
        });
        page = await browser.newPage();

        // Log browser console messages to Node console for debugging
        page.on('console', msg => {
            const type = msg.type();
            const text = msg.text();
            // Filter out less important logs if needed
            if (type === 'log' || type === 'warn' || type === 'error') {
                console.log(`PAGE CONSOLE [${type.toUpperCase()}]: ${text}`);
            }
        });
        page.on('pageerror', error => {
            console.error(`PAGE ERROR: ${error.message}`);
        });


        // Adjust the path if your test file is not in the root directory
        const filePath = `file://${path.join(__dirname, '../flipper.html')}`;
        await page.goto(filePath, { waitUntil: 'networkidle0' });
    });

    afterAll(async () => {
        if (browser) {
            await browser.close();
        }
    });

    test('should run autonomously and reach a score of 99 when balls settle below flippers', async () => {
        // Wait for the game world and our test function to be ready
        try {
            // Corrected waitForFunction to check for global `world` and `window.getGameStateForTest`
            await page.waitForFunction(() => (typeof world !== 'undefined' && world) && typeof window.getGameStateForTest === 'function', { timeout: 10000 });
        } catch (e) {
            throw new Error("Game world or getGameStateForTest function did not become available in time.");
        }

        // The game starts paused. Click the "Start" button (ID: pauseBtn)
        // Initial text is "Start" or "Resume". Clicking it starts the game.
        await page.click('#pauseBtn');
        console.log("Game started by clicking #pauseBtn.");

        const MAX_GAME_SIMULATION_TIME = 100000; // 100 seconds timeout for game simulation
        const POLLING_INTERVAL = 1000;    // ms, how often to check game state
        const FLIPPER_Y_LINE = 0.05;     // Y-coordinate just above 1 ball radius (floor is at y=0)

        let testPassed = false;
        let settled = false;
        const startTime = Date.now();

        while (Date.now() - startTime < MAX_GAME_SIMULATION_TIME) {
            const gameState = await page.evaluate(() => getGameStateForTest());

            if (!gameState) {
                // This might happen if the page context is lost or getGameStateForTest fails
                throw new Error("Failed to get game state from the page.");
            }

            const { balls, score } = gameState;

            // Log current state for debugging CI if needed
            // console.log(`Current state - Score: ${score}, Balls: ${balls.length}`);

            // Condition 1: Score must not exceed 99
            if (score > 99) {
                throw new Error(`Test failed: Score exceeded 99. Current score: ${score}`);
            }

            // Condition 2: Check if balls have settled below flippers
            let allBallsBelowFlippers = balls.length > 0; // Assume true only if there are balls
            if (balls.length === 0) {
                // This could be an issue or mean balls are removed from the game.
                // For this test, if no balls, they can't be "settled below flippers".
                allBallsBelowFlippers = false;
            } else {
                for (const ball of balls) {
                    if (ball.y === -Infinity) { // Check for invalid ball data
                        throw new Error(`Invalid ball data encountered for ball ID ${ball.id}`);
                    }
                    if (ball.y >= FLIPPER_Y_LINE) {
                        allBallsBelowFlippers = false;
                        break;
                    }
                }
            }

            if (allBallsBelowFlippers) {
                console.log(`All balls detected below flipper line (Y < ${FLIPPER_Y_LINE}). Current score: ${score}.`);
                settled = true;
                if (score === 99) {
                    testPassed = true;
                } else {
                    throw new Error(`Test failed: Balls settled below flippers, but score is ${score} (expected 99).`);
                }
                break; // Exit polling loop
            }

            await new Promise(resolve => setTimeout(resolve, POLLING_INTERVAL));
        }

        // Final assertions after the loop
        if (!settled) {
            const finalGameState = await page.evaluate(() => getGameStateForTest());
            throw new Error(`Test failed: Timeout. Balls did not settle below flippers within ${MAX_GAME_SIMULATION_TIME / 1000}s. Final score: ${finalGameState.score}`);
        }

        // This assertion is a bit redundant if the loop logic is correct, but good for clarity
        expect(testPassed).toBe(true);
        expect(settled).toBe(true); // Ensure we exited because balls settled
        const finalScore = (await page.evaluate(() => getGameStateForTest())).score;
        expect(finalScore).toBe(99); // Final check on score

    });
});
