import com.mongodb.ConnectionString;
import com.mongodb.client.MongoClients;
import com.mongodb.client.MongoClient;
import com.mongodb.client.MongoDatabase;
import com.mongodb.MongoClientSettings;
import com.mongodb.ServerAddress;
import com.mongodb.MongoCredential;
import com.mongodb.MongoClientOptions;

//import java.util.Arrays;


/* This is a simple Java program.
   FileName : "HelloWorld.java". */
class test
{
    // Your program begins with a call to main().
    // Prints "Hello, World" to the terminal window.
    public static void main(String args[])
    {
        System.out.println("Hello, World");
	ConnectionString connectionString = new ConnectionString("mongodb+srv://admin:admin@test.7eofe.mongodb.net/myFirstDatabase?retryWrites=true&w=majority");
	MongoClientSettings settings = MongoClientSettings.builder()
	    .applyConnectionString(connectionString)
	    .build();
	MongoClient mongoClient = MongoClients.create(settings);
	MongoDatabase database = mongoClient.getDatabase("test");
	System.out.println(database);
    }
}

