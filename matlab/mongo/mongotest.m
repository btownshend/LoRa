%server = "test.7eofe.mongodb.net";
server = ["test-shard-00-00.7eofe.mongodb.net",
          "test-shard-00-01.7eofe.mongodb.net",
          "test-shard-00-02.7eofe.mongodb.net"];
port = [27017,27017,27017];
dbname = 'Test';
conn = mongo(server,port,dbname,'UserName','admin','Password','admin','SSLEnabled',true,'AuthMechanism','SCRAM_SHA_1','WriteConcern','w1','ReadPreference','nearest')
collection="testcollection";
docs=find(conn,collection)