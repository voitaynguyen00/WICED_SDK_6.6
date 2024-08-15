/*
* Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor 
 *  Corporation. All rights reserved. This software, including source code, documentation and  related 
 * materials ("Software"), is owned by Cypress Semiconductor  Corporation or one of its 
 *  subsidiaries ("Cypress") and is protected by and subject to worldwide patent protection  
 * (United States and foreign), United States copyright laws and international treaty provisions. 
 * Therefore, you may use this Software only as provided in the license agreement accompanying the 
 * software package from which you obtained this Software ("EULA"). If no EULA applies, Cypress 
 * hereby grants you a personal, nonexclusive, non-transferable license to  copy, modify, and 
 * compile the Software source code solely for use in connection with Cypress's  integrated circuit 
 * products. Any reproduction, modification, translation, compilation,  or representation of this 
 * Software except as specified above is prohibited without the express written permission of 
 * Cypress. Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO  WARRANTY OF ANY KIND, EXPRESS 
 * OR IMPLIED, INCLUDING,  BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to 
 * the Software without notice. Cypress does not assume any liability arising out of the application 
 * or use of the Software or any product or circuit  described in the Software. Cypress does 
 * not authorize its products for use in any products where a malfunction or failure of the 
 * Cypress product may reasonably be expected to result  in significant property damage, injury 
 * or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the 
 *  manufacturer of such system or application assumes  all risk of such use and in doing so agrees 
 * to indemnify Cypress against all liability.
*/
package com.cypress.le.mesh.meshapp;

import android.content.Context;
import android.os.AsyncTask;
import android.support.annotation.WorkerThread;
import android.util.Log;

import com.amazonaws.SDKGlobalConfiguration;
import com.amazonaws.auth.AWSCredentialsProvider;
import com.amazonaws.auth.BasicAWSCredentials;
import com.amazonaws.internal.StaticCredentialsProvider;
import com.amazonaws.regions.Region;
import com.amazonaws.regions.Regions;
import com.amazonaws.services.iotdata.AWSIotDataClient;
import com.amazonaws.services.iotdata.model.GetThingShadowRequest;
import com.amazonaws.services.iotdata.model.GetThingShadowResult;
import com.amazonaws.services.iotdata.model.PublishRequest;


import org.apache.http.HttpResponse;
import org.apache.http.client.ClientProtocolException;
import org.apache.http.client.HttpClient;
import org.apache.http.client.ResponseHandler;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.impl.client.DefaultHttpClient;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URI;
import java.net.URISyntaxException;
import java.net.URL;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

class CloudConfigHelper {

    private Context mCtx = null;
    private static final String TAG = "CloudConfigHelper";
    private String bigIpAddr = "";
    AWSIotDataClient client;
    IHelperCallback mCallback = null;
    HttpRequest mRequest;
    HttpURLConnection mUrlConnection;
    private static final byte PKT_PTR_TO_MULTI_ACTUATORS =     (byte) 0x01;
    private static final byte PKT_PTR_TO_MULTI_PTR       =     (byte) 0x02;
    private static final byte PKT_PTR_TO_PTR             =     (byte) 0x03;


    String meshUrlBaseString;
    String urlEncodedString;
    boolean isConnectingToGateway = false;

    private String  STRING_MESH_PT2MULTIPTACTPKT = "sendactuator";
    private String  STRING_MESH_PT2MULTIPTPKT    = "sendmultipt";
    private String  STRING_MESH_PT2PT            = "sendp2p";
    private String Data;

    private static int GlobalAckRef = 0x11223344;
// ++

    String mStartPatternD="{\"state\":{\"desired\":{\"status\":\"";
    String mEndPatternD="\",\"auto\":";
    String mStartPatternR="\"reported\":{\"status\":\"";
    String mEndPatternR="\",\"auto\":";
    String mStartPatternA="\"auto\":\"";
    String mEndPatternA="\"},";
//    String mThingText = "MESH";
//    String mAccessKey = "AKIAI64XPGOM5MPQCI7Q";
//    String mSecretKey = "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";
//    String mRestShadowEndpoint = "https://A38TD4KE8SEEKY.iot.us-east-1.amazonaws.com";
    String mThingText = null;
    String mAccessKey = null;
    String mSecretKey = null;
    String mRestShadowEndpoint = null;

    boolean mPublishException = true;
    String Astate;
    String Rstate="ON";
    String Dstate="OFF";
    boolean mIsException=false;
    String payloadStr = "";
    String autoState = "ON";
// --

    String desiredState = "";
    String reportedState = "2200220022002200";
    Region region = Region.getRegion(Regions.US_EAST_1);


    // Filename of KeyStore file on the filesystem
    private static final String KEYSTORE_NAME = "iot_keystore";
    // Password for the private key in the KeyStore
    private static final String KEYSTORE_PASSWORD = "password";
    // Certificate and key aliases in the KeyStore
    private static final String CERTIFICATE_ID = "default";

    public static String toHexString(byte[] bytes) {
        int len = bytes.length;
        if (len == 0)
            return null;

        char[] buffer = new char[len * 2];

        for (int i = 0, index = 0; i < len; i++) {
//            if (i > 0) {
//                buffer[index++] = ' ';
//            }
            int data = bytes[i];
            if (data < 0) {
                data += 256;
            }

            byte n = (byte) (data >>> 4);
            if (n < 10) {
                buffer[index++] = (char) ('0' + n);
            } else {
                buffer[index++] = (char) ('A' + n - 10);
            }

            n = (byte) (data & 0x0F);
            if (n < 10) {
                buffer[index++] = (char) ('0' + n);
            } else {
                buffer[index++] = (char) ('A' + n - 10);
            }
        }
        return new String(buffer);
    }

    public boolean init(Context ctx) {
        mCtx = ctx;
        awsShadowRestInit();
//            mBluetoothManager = (BluetoothManager) mCtx.getSystemService(Context.BLUETOOTH_SERVICE);
//            if (mBluetoothManager == null)
//                return false;
//        }
//        if (mBluetoothAdapter == null) {
//            mBluetoothAdapter = mBluetoothManager.getAdapter();
//            if (mBluetoothAdapter == null)
//                return false;
//        }
        return true;
    }
    private void awsShadowRestInit() {
        if(mAccessKey != null && mSecretKey!=null &&  mRestShadowEndpoint!=null && mThingText!=null) {
            System.setProperty(SDKGlobalConfiguration.DISABLE_CERT_CHECKING_SYSTEM_PROPERTY, "true");
            // use hard coded credentials for testing only
            AWSCredentialsProvider provider = new StaticCredentialsProvider(new BasicAWSCredentials(
                    mAccessKey, mSecretKey));
            client = new AWSIotDataClient(provider);
            client.setEndpoint(mRestShadowEndpoint,"iot","us-east-1");
        } else {
            Log.d(TAG,"Cannot init aws, keys are not present");
        }

    }


    public boolean connectRest(String ipAddr){
        bigIpAddr = ipAddr;
        Log.d(TAG, "ipAddr = " + ipAddr);
        mRequest = new HttpRequest();
        mRequest.execute();
        return true;
    }



    private byte[] offsetBuffer(byte[] value, int offset) {
        if (offset >= value.length)
            return new byte[0];
        byte[] res = new byte[value.length - offset];
        for (int i = offset; i != value.length; ++i) {
            res[i - offset] = value[i];
        }
        return res;
    }

    private ResponseHandler<String> mHttpResponseHandler = new ResponseHandler<String>() {

        @Override
        public String handleResponse(final HttpResponse response)
                throws ClientProtocolException {
            int status = response.getStatusLine().getStatusCode();
            Log.d(TAG, "Status: " + status);
            if (status >= 200 && status < 300) {
                Log.i(TAG, "status is >=200 and < 300");
                Log.e(TAG,"action_sent_rest_data");
                //reset the semaphore
                try{
                    response.getEntity().getContent().close();
                }catch(IOException e){
                    e.printStackTrace();
                }

            } else {
                throw new ClientProtocolException(
                        "Unexpected response status: " + status);
            }
            return "SUCCESS";
        }

    };


    boolean send(byte[] data) {
        //meshUrlBaseString = "http://" + bigIpAddr +"/mesh/sendp2m/112233445566778899";
        if(bigIpAddr == null){
            Log.e(TAG,"send : bigIp set is null , cannot execute the command");
            return false;
        }
        Log.d(TAG, "send : bigIpAddr = " + bigIpAddr);
        //sendViaRest(data, type);
         if(bigIpAddr.equals("aws-iot")){
             sendViaAWS(data);
         } else{
             sendViaRest(data);
         }
        return true;
    }


    private void sendViaAWS(byte[] data)
    {
        Log.d(TAG, "sendViaAWS");
        desiredState = toHexString(data);
        updateShadow();
    }

    public void registerCb(IHelperCallback callback) {
        mCallback = callback;
    }

    public void connectGateway() {
        Log.d(TAG, "connectGateway");
        connectGatewayRest();
    }

    public void disconnectGateway() {
        Log.d(TAG, "disconnectGateway");
        disconnectGatewayRest();
    }

    public interface IHelperCallback {
        void onConnectionStateChange(boolean state);
        void onDataReceived(byte[] data);
    }

    private class HttpRequest extends AsyncTask {
        @Override
        protected Object doInBackground(Object[] params){
            try {
                String urlStr = "http:"+"//"+bigIpAddr+"/mesh/subscribe/sse";
                URL url = new URL(urlStr);
                mUrlConnection = (HttpURLConnection) url.openConnection();
                Log.d("SSE", "http response: " + mUrlConnection.getResponseCode());

                //Object inputStream = urlConnection.getContent();
                InputStream inputStream = new BufferedInputStream(mUrlConnection.getInputStream());

                String str = readStream(inputStream);
                Log.d("SSE reading stream", str+"");


            } catch (MalformedURLException e) {
                e.printStackTrace();
            } catch (IOException e) {
                Log.e("SSE activity", "Error on url openConnection: "+e.getMessage());
                e.printStackTrace();
            }

            return null;
        }
    }

    public static byte[] hexStringToByteArray(String s) {
        byte[] b = new byte[s.length() / 2];
        for (int i = 0; i < b.length; i++) {
            int index = i * 2;
            int v = Integer.parseInt(s.substring(index, index + 2), 16);
            b[i] = (byte) v;
        }
        return b;
    }
    String incomingPacket = "";
    private String readStream(InputStream inputStream) {
        Log.d(TAG,"ServerSentEvents SSE DATA readstream start");
        BufferedReader reader = null;
        StringBuffer response = new StringBuffer();

        try{
            reader = new BufferedReader(new InputStreamReader(inputStream));
            String line = "";
            JSONObject obj = null;
            while((line = reader.readLine()) != null){
                Log.d("ServerSentEvents 1", "SSE event: "+line);
                obj = new JSONObject(line);
                String data = obj.getString("data ");
                final byte[] databyte = parseHexBinary(data);
                Log.d(TAG,"ServerSentEvents SSE DATA : "+toHexString(databyte)+"len"+databyte.length);
                handleProxyData(databyte);
            }

        } catch (IOException e){
            e.printStackTrace();
        } catch (JSONException e) {
            e.printStackTrace();
        } finally {
            if(reader != null){
                try{
                    reader.close();
                }catch (IOException e){
                    e.printStackTrace();
                }
            }
        }

        Log.d(TAG,"ServerSentEvents SSE DATA return");
        return response.toString();
    }

    private void handleProxyData(byte[] databyte) {
        byte type = databyte[0];
        Log.d(TAG,"handleProxyData"+type);
        switch (type)
        {
            case 0x00 :
                mCallback.onConnectionStateChange(databyte[1] > 0 ?true:false);
                if(databyte[1] == 0)
                {
                    mUrlConnection.disconnect();
                    mRequest.cancel(true);
                }
                break;
            case 0x01 :
                byte[] proxypkt = new byte[databyte.length - 1];
                System.arraycopy(databyte,1,proxypkt,0,databyte.length-1);
                mCallback.onDataReceived(proxypkt);
                break;
        }
    }


    private void sendViaRest(byte[] data) {
        Log.d(TAG,"sendViaRest");
        Data = new String();
        meshUrlBaseString = "http://" + bigIpAddr +"/mesh/";
         Data = "meshdata" + "/value/" + toHexString(data);
        new Thread() {
            @Override
            public void run() {
                try {

                    Log.i(TAG, "constructed url  = " + meshUrlBaseString +Data);
                    URI uri = new URI(meshUrlBaseString + Data);
                    HttpClient httpclient = new DefaultHttpClient();
                    Log.i(TAG, "calling new HttpGet(uri) : strlen of uri = " + uri.toString().length());
                    HttpGet get = new HttpGet(uri);
                    Log.i(TAG, "calling httpclient.execute(put, mHttpResponseHandler)");
                    httpclient.execute(get, mHttpResponseHandler);


                    Log.i(TAG, "completed httpclient.execute(put, mHttpResponseHandler)");
                    //meshUrlBaseString = null;
                    get = null;
                } catch (URISyntaxException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                } catch (Exception e) {
                    Log.i(TAG, "catching exception in general");
                    e.printStackTrace();
                }
            }
        }.start();
        Log.e(TAG, "end of send");
    }

    private void connectGatewayRest() {
        Log.d(TAG,"connectGatewayRest");
        Data = new String();
        meshUrlBaseString = "http://" + bigIpAddr +"/mesh/connect";

        new Thread() {
            @Override
            public void run() {
                try {

                    Log.i(TAG, "constructed url  = " + meshUrlBaseString );
                    URI uri = new URI(meshUrlBaseString);
                    HttpClient httpclient = new DefaultHttpClient();
                    Log.i(TAG, "calling new HttpGet(uri) : strlen of uri = " + uri.toString().length());
                    HttpGet get = new HttpGet(uri);
                    Log.i(TAG, "calling httpclient.execute(put, mHttpResponseHandler)");
                    httpclient.execute(get, mHttpResponseHandler);


                    Log.i(TAG, "completed httpclient.execute(put, mHttpResponseHandler)");
                    //meshUrlBaseString = null;
                    get = null;
                } catch (URISyntaxException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                } catch (Exception e) {
                    Log.i(TAG, "catching exception in general");
                    e.printStackTrace();
                }
            }
        }.start();
        Log.e(TAG, "end of send");
    }

    private void disconnectGatewayRest() {
        Log.d(TAG,"disconnectGatewayRest");
        Data = new String();
        meshUrlBaseString = "http://" + bigIpAddr +"/mesh/disconnect";

        new Thread() {
            @Override
            public void run() {
                try {

                    Log.i(TAG, "constructed url  = " + meshUrlBaseString );
                    URI uri = new URI(meshUrlBaseString);
                    HttpClient httpclient = new DefaultHttpClient();
                    Log.i(TAG, "calling new HttpGet(uri) : strlen of uri = " + uri.toString().length());
                    HttpGet get = new HttpGet(uri);
                    Log.i(TAG, "calling httpclient.execute(put, mHttpResponseHandler)");
                    httpclient.execute(get, mHttpResponseHandler);

                    Log.i(TAG, "completed httpclient.execute(put, mHttpResponseHandler)");
                    //meshUrlBaseString = null;
                    get = null;
                } catch (URISyntaxException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                } catch (Exception e) {
                    Log.i(TAG, "catching exception in general");
                    e.printStackTrace();
                }
            }
        }.start();
        Log.e(TAG, "end of send");
    }


    public void setAWScredentials(String accessKey, String secretKey, String restShadowEndpoint, String topic) {
        Log.d(TAG,"setAWScredentials accessKey="+accessKey+ " secretKey="+secretKey+" restShadowEndpoint"+restShadowEndpoint);
        mAccessKey = accessKey;
        mSecretKey = secretKey;
        mRestShadowEndpoint = restShadowEndpoint;
        mThingText = topic;
        awsShadowRestInit();

    }

    private void updateShadow(){
        Log.d(TAG, "In updateShadow");
        AsyncTask<Long, Long, Long> execute = new AsyncTask<Long, Long, Long>() {

            @Override
            protected void onPreExecute(){
                Log.d(TAG,"onPreExecute");
            }

            @Override
            protected Long doInBackground(Long... params) {
                try {
                    if(autoState.equals("ON")){
                        autoState = "OFF";
                    } else {
                        autoState = "ON";
                    }
                    payloadStr = "{ \"state\": {\"desired\": { \"status\": \""+desiredState+"\",\"auto\": \""+autoState+"\"} ,\"reported\": { \"status\": \""+reportedState+"\",\"auto\": \""+autoState+"\" } } }";

                    Log.d(TAG,"Payload String: "+payloadStr);

                    PublishRequest request = new PublishRequest();
                    String topic="$aws/things/"+mThingText+"/shadow/update";

                    request.setTopic(topic);
                    request.setPayload(ByteBuffer.wrap(payloadStr.getBytes()));
                    request.setQos(1);

                    Log.i("publish-doInBackground", "testing");
                    Log.d(TAG, "Topic : " + topic);

                    client.publish(request);

                    GetThingShadowRequest gsr = new GetThingShadowRequest();
                    gsr.withThingName(mThingText);

                    client.setEndpoint(mRestShadowEndpoint,"iot","us-east-1");
                    GetThingShadowResult gtsr = client.getThingShadow(gsr);
                    final String state= new String(gtsr.getPayload().array());
                    Log.i("result: ", state);


                    //parse Reported string
                    Pattern pr = Pattern.compile(Pattern.quote(mStartPatternR) + "(.*?)" + Pattern.quote(mEndPatternR));
                    Matcher mr = pr.matcher(state);
                    while (mr.find()) {
                        Rstate =mr.group(1).toString();
                        System.out.println("Reported Value: "+ Rstate);
                    }


                    //Parse the Desired string
                    Pattern pd = Pattern.compile(Pattern.quote(mStartPatternD) + "(.*?)" + Pattern.quote(mEndPatternD));
                    Matcher md = pd.matcher(state);
                    while (md.find()) {
                        Dstate =md.group(1).toString();
                        Log.d(TAG, "Desired Value: " + Dstate);
                    }

                    //String subs = state.substring(33,48);
                    //Log.d(TAG, "Substring: "+subs);
                    Pattern auto = Pattern.compile(Pattern.quote(mStartPatternA) + "(.*?)" + Pattern.quote(mEndPatternA));
                    Matcher au = auto.matcher(state);
                    while (au.find()) {

                        Astate =au.group(1).toString();
                        Log.d(TAG, "Auto String: " + Astate);

                    }
                    //String parsing

                    return (long) 2;
                } catch (Exception e) {
                    mIsException=true;
                    e.printStackTrace();
                    Log.e("publish-doInBackground", e.getMessage());
                    return (long) 1;

                }
            }

            @Override
            protected void onPostExecute(Long aLong) {
                super.onPostExecute(aLong);

                Log.d(TAG, "onPostExecute");

                if (mIsException==true){
                    Log.d(TAG, "Unable to Update!");
                }
                else {
                    if (Dstate.equals("ON")) {
                        Dstate = "OFF";
                    } else {
                        Dstate = "ON";
                    }
                    if (Rstate.equals("ON")) {
                        Rstate = "OFF";
                    } else {
                        Rstate = "ON";
                    }if (Astate.equals("YES")){
                        Astate = "NO";
                    }else{
                        Astate = "YES";
                    }
                }

            }


        }.execute();
    }

    public void addRoamingNode(String dest) {

        Data = dest;

        meshUrlBaseString = "http://" + bigIpAddr +"/sigmesh";

        meshUrlBaseString += "/addroamingnode"+"/value/";
        new Thread() {
            @Override
            public void run() {
                try {

                    Log.i(TAG, "constructed url  = " + meshUrlBaseString +Data);
                    URI uri = new URI(meshUrlBaseString + Data);
                    HttpClient httpclient = new DefaultHttpClient();
                    Log.i(TAG, "calling new HttpGet(uri) : strlen of uri = " + uri.toString().length());
                    HttpGet get = new HttpGet(uri);
                    Log.i(TAG, "calling httpclient.execute(put, mHttpResponseHandler)");
                    httpclient.execute(get, mHttpResponseHandler);
                    Log.i(TAG, "completed httpclient.execute(put, mHttpResponseHandler)");
                    //meshUrlBaseString = null;
                    get = null;
                } catch (URISyntaxException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                } catch (Exception e) {
                    Log.i(TAG, "catching exception in general");
                    e.printStackTrace();
                }
            }
        }.start();
        Log.e(TAG, "end of send");
    }


    public byte[] parseHexBinary(String s) {
        final int len = s.length();

        // "111" is not a valid hex encoding.
        if( len%2 != 0 )
            throw new IllegalArgumentException("hexBinary needs to be even-length: "+s);

        byte[] out = new byte[len/2];

        for( int i=0; i<len; i+=2 ) {
            int h = hexToBin(s.charAt(i  ));
            int l = hexToBin(s.charAt(i+1));
            if( h==-1 || l==-1 )
                throw new IllegalArgumentException("contains illegal character for hexBinary: "+s);

            out[i/2] = (byte)(h*16+l);
        }

        return out;
    }

    private static int hexToBin( char ch ) {
        if( '0'<=ch && ch<='9' )    return ch-'0';
        if( 'A'<=ch && ch<='F' )    return ch-'A'+10;
        if( 'a'<=ch && ch<='f' )    return ch-'a'+10;
        return -1;
    }


}
