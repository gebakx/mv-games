using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System;

public class Graph
{
    public HashSet<string> vertices = new HashSet<string> {"a", "b", "c", "d", "e", "f"};

    public Dictionary<string,Dictionary<string,int>> edges = new Dictionary<string,Dictionary<string,int>>
        { 
            { "a", new Dictionary<string,int> { {"b", 4}, {"c", 2} } },
            { "b", new Dictionary<string,int> { {"c", 5}, {"d", 10} } },
            { "c", new Dictionary<string,int> { {"e", 3} } },
            { "d", new Dictionary<string,int> { {"f", 11} } },
            { "e", new Dictionary<string,int> { {"d", 4} } },
            { "f", new Dictionary<string,int> { } }
        };

    public Dictionary<string, int> h = new Dictionary<string, int> 
        { {"a", 20}, {"b", 18}, {"c", 12}, {"d", 10}, {"e", 9}, {"f", 0} };
}

public class shortestPath : MonoBehaviour
{
    void Start()
    {
        Graph g = new Graph();    
        Show(Dijkstra(g, "a", "f"));
        Show(Astar(g, "a", "f"));
    }

    List<string> Dijkstra(Graph g, string source, string target) 
    {
        var d = g.vertices.ToDictionary(v => v, v => 1000);
        var prev = g.vertices.ToDictionary(v => v, v => " ");
        d[source] = 0;
        HashSet<String> Q = new HashSet<string>(g.vertices);
        while (Q.Count() > 0)
        {
            string v = Q.Select(x => (d[x], x)).Min().Item2;
            Q.Remove(v);
            foreach (var pair in g.edges[v])
            {
                int alt = d[v] + pair.Value;
                if (alt < d[pair.Key]) 
                {
                    d[pair.Key] = alt;
                    prev[pair.Key] = v;
                }
            }   
        }
        List<string> path = new List<string>();
        path.Insert(0,target);
        while (prev[target] != " ")
        {
            target = prev[target];
            path.Insert(0,target);
        }
        return path;
    }

    List<string> Astar(Graph g, string source, string target) 
    {
        var d = g.vertices.ToDictionary(v => v, v => 1000);
        var prev = g.vertices.ToDictionary(v => v, v => " ");
        d[source] = 0;
        HashSet<String> Q = new HashSet<string>(g.vertices);
        while (Q.Count() > 0)
        {
            string v = Q.Select(x => (d[x] + g.h[x], x)).Min().Item2;
            Q.Remove(v);
            foreach (var pair in g.edges[v])
            {
                int alt = d[v] + pair.Value;
                if (alt < d[pair.Key]) 
                {
                    d[pair.Key] = alt;
                    prev[pair.Key] = v;
                }
            }   
        }

        List<string> path = new List<string>();
        path.Insert(0,target);
        while (prev[target] != " ")
        {
            target = prev[target];
            path.Insert(0,target);
        }
        return path;
    }

    void Show(List<string> l)
    {
        string s = "Path:\n";
        foreach (var x in l)
            s += " " + x;
        Debug.Log(s);        
    }
}
