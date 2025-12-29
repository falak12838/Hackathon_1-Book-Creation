// URDF validation test
// File: tests/urdf/test_urdf-validation.js

const fs = require('fs');
const path = require('path');

function validateURDFFiles() {
    const urdfFilesToCheck = [
        path.join(__dirname, '../../docs/examples/simple-humanoid.urdf')
    ];

    let allValid = true;

    for (const filePath of urdfFilesToCheck) {
        if (!fs.existsSync(filePath)) {
            console.error(`FAIL: URDF file does not exist: ${filePath}`);
            allValid = false;
            continue;
        }

        const content = fs.readFileSync(filePath, 'utf8');

        // Check if it looks like a URDF file (has basic XML structure and robot tag)
        if (!content.includes('<robot') || !content.includes('</robot>')) {
            console.error(`FAIL: File is not a valid URDF: ${filePath}`);
            allValid = false;
            continue;
        }

        // Check for essential URDF elements
        const essentialElements = ['<link', '<joint'];
        for (const element of essentialElements) {
            if (!content.includes(element)) {
                console.error(`FAIL: URDF missing essential element: ${element} in ${filePath}`);
                allValid = false;
            }
        }
    }

    if (allValid) {
        console.log('PASS: URDF validation');
    }

    return allValid;
}

// Run the test
const result = validateURDFFiles();
process.exit(result ? 0 : 1);