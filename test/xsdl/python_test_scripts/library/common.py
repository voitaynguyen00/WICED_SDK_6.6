# Common Automation functions

def end_test( wiced, test_result ):
    print "Exiting script\n"
    wiced.close()
    print "---------- TEST RESULT = %s\n" % test_result
    quit()
    
def end_e2e_test( wiced1=None, wiced2=None, test_result=None ):
    print "Exiting script\n"
    if ( wiced1 != None ):
        if ( wiced1.started == "true" ):
            print "*** wiced1.close() ***\n"
            wiced1.close()
    else:
        print "*** wiced1 is None! ***\n"
    if ( wiced2 != None ):
        if ( wiced2.started == "true" ):
            print "*** wiced2.close() ***\n"
            wiced2.close()
    else:
        print "*** wiced2 is None! ***\n"
        
    print "---------- TEST RESULT = %s\n" % test_result
    quit()
    
    
