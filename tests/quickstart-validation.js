// Quickstart validation test
// File: tests/quickstart-validation.js

const fs = require('fs');
const path = require('path');

function validateQuickstart() {
    const quickstartPath = path.join(__dirname, '../specs/1-robotic-nervous-system/quickstart.md');

    if (!fs.existsSync(quickstartPath)) {
        console.error('FAIL: quickstart.md file does not exist');
        return false;
    }

    const content = fs.readFileSync(quickstartPath, 'utf8');

    // Check for essential sections
    const requiredSections = [
        'Prerequisites',
        'Setup Instructions',
        'Adding Content',
        'Building for Production'
    ];

    for (const section of requiredSections) {
        if (!content.includes(section)) {
            console.error(`FAIL: Missing section '${section}' in quickstart.md`);
            return false;
    }
    }

    // Check for essential commands
    const requiredCommands = [
        'npm install',
        'npm run start',
        'npm run build'
    ];

    for (const command of requiredCommands) {
        if (!content.includes(command)) {
            console.error(`FAIL: Missing command '${command}' in quickstart.md`);
            return false;
        }
    }

    console.log('PASS: Quickstart validation');
    return true;
}

// Run the validation
const result = validateQuickstart();
process.exit(result ? 0 : 1);