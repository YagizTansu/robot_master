#!/bin/bash

# Enable changeStreamPreAndPostImages for prohibited_zones collection
# This allows change streams to capture deleted documents

echo "🔧 Configuring MongoDB collection for delete tracking..."

mongosh << 'EOF'
use robot_database

// Drop and recreate collection with changeStreamPreAndPostImages
db.prohibited_zones.drop()

db.createCollection("prohibited_zones", {
  changeStreamPreAndPostImages: { enabled: true }
})

print("✅ Collection 'prohibited_zones' created with preImage support")

// Verify configuration
var collInfo = db.getCollectionInfos({name: "prohibited_zones"})[0]
print("\nCollection options:")
printjson(collInfo.options)

EOF

echo "✅ Done! Now delete operations will be tracked properly."
