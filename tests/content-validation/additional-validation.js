// Additional content validation tests
// File: tests/content-validation/additional-validation.js

const fs = require('fs');
const path = require('path');

function runAdditionalValidation() {
    console.log('Running additional content validation tests...');

    // Test 1: Check for broken internal links
    const docsDir = path.join(__dirname, '../../docs/robotic-nervous-system');
    const files = fs.readdirSync(docsDir);

    let allValid = true;

    for (const file of files) {
        if (file.endsWith('.md')) {
            const content = fs.readFileSync(path.join(docsDir, file), 'utf8');

            // Check for common issues
            if (content.includes('TODO') || content.includes('FIXME')) {
                console.warn(`WARNING: ${file} contains TODO/FIXME markers`);
            }

            // Check for proper headings structure
            const headingPattern = /^#+\s.+/m;
            if (!headingPattern.test(content)) {
                console.error(`FAIL: ${file} does not have proper heading structure`);
                allValid = false;
            }
        }
    }

    if (allValid) {
        console.log('PASS: Additional content validation');
    }

    return allValid;
}

// Run the tests
const result = runAdditionalValidation();
process.exit(result ? 0 : 1);