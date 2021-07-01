port=27017;
if false
  server = "test.7eofe.mongodb.net";
else
  server = ["test-shard-00-00.7eofe.mongodb.net",
            "test-shard-00-01.7eofe.mongodb.net",
            "test-shard-00-02.7eofe.mongodb.net"];
  port(1:length(server))=port(1);
end
dbname = 'admin';
conn = mongo(server,port,dbname,'UserName','admin','Password','admin','SSLEnabled',true,'AuthMechanism','SCRAM_SHA_1','WriteConcern','w1','ReadPreference','nearest');
conn.Database='Test';
collection="rx";
ndocs=count(conn,collection);
fprintf('Collection %s has %d documents\n',collection, ndocs);
docs=find(conn,collection,'Skip',ndocs-200);
