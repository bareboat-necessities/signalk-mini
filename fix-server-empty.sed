# Replace unsupported ConnectionRegistry::empty() call.
s/!connections\.empty()/connections.size() != 0/
