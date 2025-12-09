#!/bin/bash
# Quick push script - run this after adding SSH key to GitHub

cd /home/litahsu/AAE5303_assignment2_openvins_demo

echo "Testing GitHub connection..."
ssh -T git@github.com 2>&1 | grep -q "successfully authenticated" && {
    echo "✓ SSH authentication successful!"
    echo "Pushing to GitHub..."
    git push -u origin master
    echo ""
    echo "✅ Code pushed successfully!"
    echo "View at: https://github.com/qmohsu/AAE5303_assignment2_openvins_demo"
    exit 0
}

echo "❌ SSH not configured yet."
echo ""
echo "Please add this SSH key to GitHub:"
echo ""
cat ~/.ssh/id_ed25519.pub
echo ""
echo "Go to: https://github.com/settings/keys"
echo "Then run this script again!"

